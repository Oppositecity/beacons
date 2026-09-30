// BEACONS v0.8 — the air around you as a small band, for M5Stack Cardputer (original + ADV)
//
// Everything plays in one key over a slowly turning chord progression, and every note sounds
// the moment its signal arrives. Each signal the radio hears takes a musical role:
//   1 beacons    WiFi access points announcing themselves   -> marimba, middle voices
//   2 probes     phones calling for networks they remember  -> kalimba, high voices
//   3 traffic    WiFi data frames on this channel           -> vibraphone arpeggio, a note every Nth frame
//   4 floor      the radio noise floor                      -> drone on the chord root, gliding with each change
//   5 deauth     disconnect frames (rare; often an attack)  -> a struck bell chord
//   6 bluetooth  BLE devices advertising nearby             -> tongue drum bass, roots and fifths
//   7 motion     shaking the Cardputer (ADV motion sensor)  -> a strummed chord
// Access points keep their own drifting clocks, so their pulses phase against each other.
// Tempo sets how fast the chords change.
//
// Synthesis is modal (struck-bar physics, natural decays, soft onsets). Pitches stay within
// 130–2000 Hz, the output passes a soft limiter, and volume is capped.
//
// Two screens, switched with tab:
//   play   up to six voices as rows, highest pitch at top; a row flashes when it sounds,
//          the bar is signal strength. ? = probe, * = bluetooth.
//   mixer  the seven layers; UPPERCASE = on, the bar is level, a row flashes when that layer sounds.
//
// keys (both screens):  1-7 layer on/off   tab switch screen   m mute   q sleep (reset to wake)
// play screen:          , / channel   h hop   ; . octave   t chord tempo   s mood (chord progression)
//                       l note length   [ ] thin/thicken   - = volume
// mixer screen:         ; . select layer   - = layer level
//
// Nothing is stored or transmitted. Listening only.

#include <M5Cardputer.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// =====================================================================
//  types — all defined before the first function, so the Arduino
//  builder's auto-generated prototypes always compile
// =====================================================================
enum { L_BEACON, L_PROBE, L_TRAFFIC, L_FLOOR, L_DEAUTH, L_BLE, L_MOTION, NLAYERS };
enum { T_MARIMBA, T_KALIMBA, T_TONGUE, T_VIBES, T_BELL, NTIMBRES };
struct Ev { uint8_t kind; uint8_t mac[6]; int8_t rssi; char name[33]; };   // kind 0 AP, 1 probe, 2 BLE
struct Partial { float a1, a2, y1, y2, env, rblk; };
struct Note { Partial p[3]; int np; bool active; };
struct NoteEv { float f, amp, len; uint8_t timbre; };
struct Voice {
  uint8_t mac[6]; char s[33]; uint8_t kind; int rssi;
  uint32_t seen, flash, lastSound, n; bool used;
};
struct Sched { uint32_t at; float f, amp, len; uint8_t timbre, layer; int16_t voice; bool used; };

// =====================================================================
//  layers and controls
// =====================================================================
static const char* LAYER_NAMES[NLAYERS] = {"beacons", "probes", "traffic", "floor", "deauth", "bluetooth", "motion"};
static bool     layerOn[NLAYERS]    = {true, true, true, true, true, true, true};
static int      layerLvl[NLAYERS]   = {7, 5, 5, 4, 6, 6, 6};     // 0..10
static uint32_t layerFlash[NLAYERS] = {0};
static bool     imuOk = false;

static int  channel  = 6;
static bool muted    = false;
static bool hopping  = false;
static int  octave   = 0;
static int  moodIdx  = 0;
static int  tempoIdx = 2;
static int  lenIdx   = 1;
static int  volume   = 120;
static const int VOL_MAX = 200;                // hard cap for ears and equipment
static int  divIdx   = 3;
static bool mixerView = false;
static int  sel      = 0;
static const float LENS[] = {0.5f, 1.0f, 2.0f, 4.0f};
static const int DIVS[] = {1, 2, 5, 10, 20, 50};
static const int TEMPOS[] = {60, 72, 84, 96, 108};

// four moods, each a four-chord progression; chords are semitones above the key root
static const char* MOOD_NAMES[] = {"major", "dorian", "lydian", "minor"};
static const int CHORDS[4][4][4] = {
  {{0, 4, 7, 14}, {9, 12, 16, 19}, {5, 9, 12, 16}, {7, 12, 14, 19}},   // I add9, vi7, IV maj7, V sus
  {{0, 3, 7, 14}, {5, 9, 12, 15}, {0, 3, 7, 10}, {10, 14, 17, 22}},    // i9, IV7, i7, bVII
  {{0, 4, 7, 11}, {2, 6, 9, 13}, {0, 4, 7, 14}, {2, 6, 9, 16}},        // I maj7, II, I add9, II add9
  {{0, 3, 7, 10}, {8, 12, 15, 19}, {3, 7, 10, 14}, {10, 14, 17, 21}},  // i7, VI maj7, III maj7, bVII
};
static int chordIdx = 0;

// =====================================================================
//  radio -> loop event queue
// =====================================================================
static const int QN = 64;
static Ev q[QN];
static volatile int qHead = 0, qTail = 0;
static portMUX_TYPE qmux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t dataCount = 0, deauthCount = 0;
static volatile int noiseFloor = -95;

static void pushEv(const Ev& e) {
  portENTER_CRITICAL(&qmux);
  int next = (qHead + 1) % QN;
  if (next != qTail) { q[qHead] = e; qHead = next; }
  portEXIT_CRITICAL(&qmux);
}
static bool popEv(Ev& e) {
  bool ok = false;
  portENTER_CRITICAL(&qmux);
  if (qTail != qHead) { e = q[qTail]; qTail = (qTail + 1) % QN; ok = true; }
  portEXIT_CRITICAL(&qmux);
  return ok;
}

// WiFi promiscuous callback — runs in the wifi task, keep it tiny
static void onPkt(void* buf, wifi_promiscuous_pkt_type_t type) {
  auto* p = (wifi_promiscuous_pkt_t*)buf;
  noiseFloor = p->rx_ctrl.noise_floor;
  if (type == WIFI_PKT_DATA) { dataCount++; return; }
  if (type != WIFI_PKT_MGMT) return;
  const uint8_t* f = p->payload;
  int len = p->rx_ctrl.sig_len;
  if (len < 24) return;
  uint8_t sub = (f[0] >> 4) & 0x0F;
  if (sub == 12 || sub == 10) { deauthCount++; return; }   // deauth / disassoc
  uint8_t kind; int tag;
  if (sub == 8)      { kind = 0; tag = 36; }   // beacon
  else if (sub == 4) { kind = 1; tag = 24; }   // probe request
  else return;
  Ev e;
  e.kind = kind;
  e.rssi = p->rx_ctrl.rssi;
  memcpy(e.mac, f + 10, 6);
  e.name[0] = 0;
  if (len > tag + 2 && f[tag] == 0) {
    int sl = f[tag + 1]; if (sl > 32) sl = 32;
    if (tag + 2 + sl <= len) { memcpy(e.name, f + tag + 2, sl); e.name[sl] = 0; }
  }
  pushEv(e);
}

// BLE advertisement callback — runs in the bluetooth task
class BleCb : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    Ev e;
    e.kind = 2;
    e.rssi = d.getRSSI();
    memcpy(e.mac, *d.getAddress().getNative(), 6);
    e.name[0] = 0;
    if (d.haveName()) { strncpy(e.name, d.getName().c_str(), 32); e.name[32] = 0; }
    pushEv(e);
  }
};
static BLEScan* bleScan = nullptr;
static volatile bool bleScanning = false;
static void bleDone(BLEScanResults) { bleScanning = false; }

// =====================================================================
//  harmony
// =====================================================================
static uint32_t hashMac(const uint8_t* m) {
  uint32_t h = 2166136261u;
  for (int i = 0; i < 6; i++) { h ^= m[i]; h *= 16777619u; }
  return h;
}
static float fold(float f, float lo, float hi) {
  while (f < lo) f *= 2.0f;
  while (f > hi) f *= 0.5f;
  return f;
}
static float keyRoot() { return 130.81f * powf(2.0f, octave); }           // C3, shifted by octave
static float semi(int s) { return keyRoot() * powf(2.0f, s / 12.0f); }
static int chordTone(int i) { return CHORDS[moodIdx][chordIdx][i & 3]; }

// every device has a fixed role in the chord: the chord changes, the role stays
static float pitchFor(const uint8_t* mac, int kind) {
  uint32_t h = hashMac(mac);
  if (kind == 2) return fold(semi(chordTone((h % 2) * 2)), 130, 330);                  // bass: root or fifth
  if (kind == 1) return fold(semi(chordTone(h % 4) + 24 + 12 * ((h >> 8) % 2)), 520, 2000);  // high
  return fold(semi(chordTone(h % 4) + 12 + 12 * ((h >> 8) % 2)), 196, 1100);           // middle
}

// =====================================================================
//  synthesis engine — modal resonators in their own task
// =====================================================================
static const int SR = 22050, BLK = 256;
static const int NNOTES = 24;
static Note notes[NNOTES];
static const int NQ = 32;
static NoteEv nq[NQ];
static volatile int nqHead = 0, nqTail = 0;
static portMUX_TYPE nmux = portMUX_INITIALIZER_UNLOCKED;

static void play(float f, float amp, uint8_t timbre, float len) {
  if (muted || amp <= 0) return;
  portENTER_CRITICAL(&nmux);
  int next = (nqHead + 1) % NQ;
  if (next != nqTail) { nq[nqHead] = {f, amp, len, timbre}; nqHead = next; }
  portEXIT_CRITICAL(&nmux);
}

// partial ratios, relative amplitudes, decay times (seconds to fade 60 dB)
static const float T_RATIO[NTIMBRES][3] = {{1, 3.99f, 9.9f}, {1, 5.9f, 0}, {1, 2.0f, 3.01f}, {1, 4.0f, 0}, {1, 2.0f, 3.0f}};
static const float T_AMP[NTIMBRES][3]   = {{1, 0.28f, 0.07f}, {1, 0.16f, 0}, {1, 0.33f, 0.09f}, {1, 0.12f, 0}, {1, 0.45f, 0.22f}};
static const float T_T60[NTIMBRES][3]   = {{1.1f, 0.22f, 0.06f}, {1.8f, 0.25f, 0}, {1.5f, 0.5f, 0.2f}, {2.6f, 0.6f, 0}, {3.5f, 2.0f, 1.2f}};

static void startNote(const NoteEv& e) {
  int slot = -1; float quietest = 1e9;
  for (int i = 0; i < NNOTES; i++) {
    if (!notes[i].active) { slot = i; quietest = 0; break; }
    if (notes[i].p[0].env < quietest) { quietest = notes[i].p[0].env; slot = i; }
  }
  if (quietest > 0.004f) return;                      // never cut off a ringing note: that clicks
  Note& n = notes[slot];
  n.np = 0;
  for (int k = 0; k < 3; k++) {
    float r = T_RATIO[e.timbre][k];
    if (r == 0) break;
    float fk = e.f * r;
    if (fk > SR * 0.42f) break;                       // nothing near Nyquist, nothing harsh
    float t60 = T_T60[e.timbre][k] * e.len;
    float rr = powf(0.001f, 1.0f / (t60 * SR));
    float w = 2.0f * PI * fk / SR;
    float A = e.amp * T_AMP[e.timbre][k];
    Partial& p = n.p[n.np++];
    p.a1 = 2.0f * rr * cosf(w); p.a2 = -rr * rr;
    p.y1 = A * sinf(w); p.y2 = 0;                     // starts at zero crossing: no click
    p.env = A; p.rblk = powf(rr, BLK);
  }
  n.active = n.np > 0;
}

// drone (floor layer): root + fifth + octave, gliding to each new chord root
static volatile float droneTarget = 0, droneRoot = 130.81f;

static float softclip(float x) {
  if (x > 3) x = 3; else if (x < -3) x = -3;
  return x * (27 + x * x) / (27 + 9 * x * x);
}

static int16_t outbuf[3][BLK];
static float mix[BLK];

static void synthTask(void*) {
  int ob = 0;
  float d1 = 0, d2 = 0, d3 = 0, lfo = 0, droneGain = 0, rootS = 130.81f;
  for (;;) {
    if (M5Cardputer.Speaker.isPlaying(0) >= 2) { vTaskDelay(1); continue; }

    portENTER_CRITICAL(&nmux);
    NoteEv pending[NQ]; int np = 0;
    while (nqTail != nqHead) { pending[np++] = nq[nqTail]; nqTail = (nqTail + 1) % NQ; }
    portEXIT_CRITICAL(&nmux);
    for (int i = 0; i < np; i++) startNote(pending[i]);

    memset(mix, 0, sizeof(mix));

    for (int i = 0; i < NNOTES; i++) {
      Note& n = notes[i];
      if (!n.active) continue;
      bool alive = false;
      for (int k = 0; k < n.np; k++) {
        Partial& p = n.p[k];
        float y1 = p.y1, y2 = p.y2, a1 = p.a1, a2 = p.a2;
        for (int s = 0; s < BLK; s++) {
          float y = a1 * y1 + a2 * y2;
          y2 = y1; y1 = y;
          mix[s] += y;
        }
        p.y1 = y1; p.y2 = y2; p.env *= p.rblk;
        if (p.env > 1e-4f) alive = true;
      }
      n.active = alive;
    }

    float dt = droneTarget, rt = droneRoot;
    for (int s = 0; s < BLK; s++) {
      droneGain += (dt - droneGain) * 0.0004f;
      rootS += (rt - rootS) * 0.00015f;                // portamento between chord roots
      if (droneGain > 1e-5f) {
        lfo += 2 * PI * 0.07f / SR; if (lfo > 2 * PI) lfo -= 2 * PI;
        float breath = 0.7f + 0.3f * sinf(lfo);
        mix[s] += droneGain * breath * (sinf(d1) + 0.55f * sinf(d2) + 0.22f * sinf(d3));
      }
      d1 += 2 * PI * rootS / SR;
      d2 += 2 * PI * (rootS * 1.5f + 0.35f) / SR;
      d3 += 2 * PI * (rootS * 2.0f - 0.2f) / SR;
      if (d1 > 2 * PI) d1 -= 2 * PI; if (d2 > 2 * PI) d2 -= 2 * PI; if (d3 > 2 * PI) d3 -= 2 * PI;
    }

    int16_t* o = outbuf[ob];
    for (int s = 0; s < BLK; s++) o[s] = (int16_t)(softclip(mix[s] * 1.8f) * 21000.0f);
    M5Cardputer.Speaker.playRaw(o, BLK, SR, false, 1, 0, false);
    ob = (ob + 1) % 3;
  }
}

// =====================================================================
//  notes sound the moment their signal arrives; a short memory keeps
//  identical notes from stacking and caps how many start at once
// =====================================================================
static const int NS = 48;
static Sched sched[NS];
static const int NRECENT = 16;
static float recentF[NRECENT];
static uint32_t recentT[NRECENT];
static int recentI = 0;

static void schedule(uint32_t at, float f, float amp, uint8_t timbre, float len, uint8_t layer, int16_t voice) {
  if (amp <= 0 || muted) return;
  int crowd = 0;
  for (int i = 0; i < NRECENT; i++) {
    if (at - recentT[i] > 60 || recentT[i] > at) continue;
    if (fabsf(recentF[i] - f) < 1.0f) return;         // the same pitch twice within 60 ms: once is enough
    crowd++;
  }
  if (crowd >= 5) return;                             // at most five notes starting together, so it never smears
  recentF[recentI] = f; recentT[recentI] = at; recentI = (recentI + 1) % NRECENT;
  for (int i = 0; i < NS; i++)
    if (!sched[i].used) { sched[i] = {at, f, amp, len, timbre, layer, voice, true}; return; }
}

// =====================================================================
//  voices on screen: one per device heard
// =====================================================================
static const int NV = 32;
static Voice voices[NV];

M5Canvas canvas(&M5Cardputer.Display);

static bool printable(char* s) {
  bool any = false;
  for (char* c = s; *c; c++) {
    if (*c < 32 || *c > 126) *c = '.';
    else if (*c != ' ') any = true;
  }
  return any;
}

static int hear(const Ev& e) {
  int slot = -1, oldest = 0;
  for (int i = 0; i < NV; i++) {
    if (voices[i].used && voices[i].kind == e.kind && memcmp(voices[i].mac, e.mac, 6) == 0) { slot = i; break; }
    if (!voices[i].used) { if (slot < 0) slot = i; }
    else if (voices[i].seen < voices[oldest].seen) oldest = i;
  }
  if (slot < 0) slot = oldest;
  Voice& v = voices[slot];
  if (!v.used || v.kind != e.kind || memcmp(v.mac, e.mac, 6) != 0) {
    memcpy(v.mac, e.mac, 6); v.kind = e.kind; v.rssi = e.rssi;
    v.n = esp_random() % 50; v.flash = 0; v.lastSound = 0; v.used = true; v.s[0] = 0;
  }
  char s[33]; strcpy(s, e.name);
  if (printable(s)) strcpy(v.s, s);
  else if (!v.s[0]) {
    if (e.kind == 1) strcpy(v.s, "any");
    else snprintf(v.s, sizeof(v.s), "%s %02x%02x", e.kind == 2 ? "ble" : "hidden", e.mac[4], e.mac[5]);
  }
  v.rssi = (v.rssi * 3 + e.rssi) / 4;
  v.seen = millis();
  return slot;
}

static void setChannel(int ch) {
  channel = ch;
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  for (int i = 0; i < NV; i++) if (voices[i].kind != 2) voices[i].used = false;
}

static float lvl(int L) { return layerOn[L] ? layerLvl[L] / 10.0f : 0.0f; }

static void goToSleep() {
  esp_wifi_set_promiscuous(false);
  M5Cardputer.Speaker.stop();
  canvas.fillSprite(TFT_BLACK);
  canvas.setTextColor(TFT_WHITE);
  canvas.setFont(&fonts::FreeMono9pt7b);
  canvas.drawString("sleeping", 70, 50);
  canvas.drawString("reset to wake", 45, 72);
  canvas.pushSprite(0, 0);
  delay(1200);
  M5Cardputer.Display.sleep();
  esp_deep_sleep_start();
}

static void handleKeys() {
  if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) return;
  auto ks = M5Cardputer.Keyboard.keysState();
  if (ks.tab) mixerView = !mixerView;
  for (char c : ks.word) {
    if (c >= '1' && c <= '7') { int L = c - '1'; if (L != L_MOTION || imuOk) layerOn[L] = !layerOn[L]; continue; }
    if (c == 'q') goToSleep();
    else if (c == 'm') muted = !muted;
    else if (mixerView) {
      if (c == ';') sel = (sel + NLAYERS - 1) % NLAYERS;
      else if (c == '.') sel = (sel + 1) % NLAYERS;
      else if (c == '=') layerLvl[sel] = min(layerLvl[sel] + 1, 10);
      else if (c == '-') layerLvl[sel] = max(layerLvl[sel] - 1, 0);
    } else {
      if (c == '/') setChannel(channel % 13 + 1);
      else if (c == ',') setChannel((channel + 11) % 13 + 1);
      else if (c == 'h') hopping = !hopping;
      else if (c == '[') divIdx = min(divIdx + 1, 5);
      else if (c == ']') divIdx = max(divIdx - 1, 0);
      else if (c == ';') octave = min(octave + 1, 1);
      else if (c == '.') octave = max(octave - 1, -1);
      else if (c == 's') moodIdx = (moodIdx + 1) % 4;
      else if (c == 't') tempoIdx = (tempoIdx + 1) % 5;
      else if (c == 'l') lenIdx = (lenIdx + 1) % 4;
      else if (c == '=') { volume = min(volume + 20, VOL_MAX); M5Cardputer.Speaker.setVolume(volume); }
      else if (c == '-') { volume = max(volume - 20, 0);       M5Cardputer.Speaker.setVolume(volume); }
    }
  }
}

// =====================================================================
//  screens
// =====================================================================
static void drawPlay(uint32_t now) {
  const int ROWS = 6, ROW_H = 17, BAR_X = 150, BAR_W = 86;
  int pick[ROWS], np = 0;
  bool taken[NV] = {false};
  for (int r = 0; r < ROWS; r++) {
    int best = -1;
    for (int i = 0; i < NV; i++) {
      Voice& v = voices[i];
      if (!v.used || taken[i] || now - v.seen > 5000) continue;
      int L = v.kind == 0 ? L_BEACON : (v.kind == 1 ? L_PROBE : L_BLE);
      if (!layerOn[L]) continue;
      if (best < 0 || v.rssi > voices[best].rssi) best = i;
    }
    if (best < 0) break;
    taken[best] = true; pick[np++] = best;
  }
  for (int a = 0; a < np; a++)
    for (int b = a + 1; b < np; b++)
      if (pitchFor(voices[pick[b]].mac, voices[pick[b]].kind) > pitchFor(voices[pick[a]].mac, voices[pick[a]].kind)) {
        int t = pick[a]; pick[a] = pick[b]; pick[b] = t;
      }

  canvas.setFont(&fonts::FreeMono9pt7b);
  for (int r = 0; r < np; r++) {
    Voice& v = voices[pick[r]];
    int y = 2 + r * ROW_H;
    bool lit = (now - v.flash < 140);
    uint16_t fg = lit ? TFT_BLACK : TFT_WHITE;
    if (lit) canvas.fillRect(0, y - 1, 240, ROW_H - 1, TFT_WHITE);
    canvas.setTextColor(fg);
    char label[14];
    snprintf(label, sizeof(label), "%s%.12s", v.kind == 1 ? "?" : (v.kind == 2 ? "*" : ""), v.s);
    canvas.drawString(label, 4, y);
    int w = map(constrain(v.rssi, -95, -30), -95, -30, 2, BAR_W);
    canvas.fillRect(BAR_X, y + 5, w, 4, fg);
  }

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(TFT_WHITE);
  char l1[48], l2[48], l3[48];
  snprintf(l1, sizeof(l1), ",/ ch%-2d  h %s  ;. oct%+d  t %d", channel, hopping ? "HOP" : "hop", octave, TEMPOS[tempoIdx]);
  snprintf(l2, sizeof(l2), "s %-6s l x%g  [] /%d  -= vol%d", MOOD_NAMES[moodIdx], LENS[lenIdx], DIVS[divIdx], volume);
  snprintf(l3, sizeof(l3), "tab mixer  1-7 layers  m %s  q sleep", muted ? "MUTED" : "mute");
  canvas.drawString(l1, 4, 104);
  canvas.drawString(l2, 4, 114);
  canvas.drawString(l3, 4, 124);
}

static void drawMixer(uint32_t now) {
  const int ROW_H = 14, BAR_X = 150;
  canvas.setFont(&fonts::FreeMono9pt7b);
  for (int L = 0; L < NLAYERS; L++) {
    int y = 2 + L * ROW_H;
    bool lit = layerOn[L] && (now - layerFlash[L] < 140);
    uint16_t fg = lit ? TFT_BLACK : TFT_WHITE;
    if (lit) canvas.fillRect(0, y - 1, 240, ROW_H, TFT_WHITE);
    canvas.setTextColor(fg);
    char name[16];
    strcpy(name, LAYER_NAMES[L]);
    if (layerOn[L]) for (char* c = name; *c; c++) *c = toupper(*c);
    char row[24];
    if (L == L_MOTION && !imuOk) snprintf(row, sizeof(row), "%c%d n/a", sel == L ? '>' : ' ', L + 1);
    else snprintf(row, sizeof(row), "%c%d %s", sel == L ? '>' : ' ', L + 1, name);
    canvas.drawString(row, 2, y);
    canvas.fillRect(BAR_X, y + 4, layerLvl[L] * 8 + 2, 4, fg);
  }
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(TFT_WHITE);
  char l2[48];
  snprintf(l2, sizeof(l2), "tab play  m %s  q sleep", muted ? "MUTED" : "mute");
  canvas.drawString("1-7 on/off   ;. select   -= level", 4, 112);
  canvas.drawString(l2, 4, 124);
}

static void draw() {
  canvas.fillSprite(TFT_BLACK);
  uint32_t now = millis();
  if (mixerView) drawMixer(now); else drawPlay(now);
  canvas.pushSprite(0, 0);
}

// =====================================================================
//  setup / loop
// =====================================================================
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Speaker.setVolume(volume);
  canvas.createSprite(240, 135);
  imuOk = M5.Imu.isEnabled();
  if (!imuOk) layerOn[L_MOTION] = false;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA };
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&onPkt);
  esp_wifi_set_promiscuous(true);
  setChannel(channel);

  BLEDevice::init("");
  bleScan = BLEDevice::getScan();
  bleScan->setAdvertisedDeviceCallbacks(new BleCb(), true);
  bleScan->setActiveScan(false);                // listen only, never ask devices for more
  bleScan->setInterval(160);
  bleScan->setWindow(60);                       // share the radio with WiFi

  xTaskCreatePinnedToCore(synthTask, "synth", 6144, nullptr, 5, nullptr, 1);
}

void loop() {
  M5Cardputer.update();
  handleKeys();
  uint32_t now = millis();

  uint32_t stepMs = 60000 / TEMPOS[tempoIdx] / 4;           // one 16th note; chords change every 32
  uint32_t step = now / stepMs;
  chordIdx = (step / 32) % 4;

  // radio events -> notes, right now
  Ev e;
  int budget = 32;
  while (budget-- && popEv(e)) {
    int L = e.kind == 0 ? L_BEACON : (e.kind == 1 ? L_PROBE : L_BLE);
    if (!layerOn[L]) continue;
    int vi = hear(e);
    Voice& v = voices[vi];
    float near = (constrain((int)e.rssi, -90, -30) + 90) / 60.0f;   // 0 far .. 1 close
    float f = pitchFor(v.mac, v.kind);
    if (e.kind == 0) {
      if (++v.n % DIVS[divIdx]) continue;                  // each AP on its own real clock, thinned
      schedule(now, f, (0.05f + 0.22f * near) * lvl(L_BEACON), T_MARIMBA, LENS[lenIdx], L_BEACON, vi);
    } else if (e.kind == 1) {
      if (now - v.lastSound < 600) continue;
      schedule(now, f, 0.12f * lvl(L_PROBE), T_KALIMBA, LENS[lenIdx], L_PROBE, vi);
    } else {
      if (now - v.lastSound < 120u * DIVS[divIdx]) continue;
      schedule(now, f, (0.06f + 0.2f * near) * lvl(L_BLE), T_TONGUE, LENS[lenIdx], L_BLE, vi);
    }
    v.lastSound = now;
  }

  // traffic: one arpeggio note every Nth data frame, the moment that frame arrives
  static uint32_t lastData = 0, lastArp = 0, lastDeauth = 0, lastBell = 0, lastStrum = 0;
  static float shake = 0;
  static int arp = 0;
  uint32_t dc = dataCount;
  uint32_t framesPerNote = 8 * DIVS[divIdx];
  if (dc - lastData >= framesPerNote) {
    lastData = dc - (dc - lastData) % framesPerNote;
    if (layerOn[L_TRAFFIC] && now - lastArp >= 90) {
      static const int ARP[8] = {0, 1, 2, 3, 2, 1, 3, 2};
      int i = ARP[arp % 8]; int up = (arp / 8) % 2 ? 12 : 0; arp++;
      float f = fold(semi(chordTone(i) + 12 + up), 260, 1400);
      schedule(now, f, 0.11f * lvl(L_TRAFFIC), T_VIBES, LENS[lenIdx], L_TRAFFIC, -1);
      lastArp = now;
    }
  }

  // floor: the drone follows the chord root and swells with the noise floor
  float nf = constrain((noiseFloor + 97) / 15.0f, 0.0f, 1.0f);
  droneRoot = fold(semi(chordTone(0)), 130, 260);
  droneTarget = muted ? 0 : (0.6f + 0.4f * nf) * 0.05f * lvl(L_FLOOR);
  static int lastNf = -95;                                    // the floor row flashes when the noise floor shifts
  if (abs(noiseFloor - lastNf) >= 2) { lastNf = noiseFloor; if (layerOn[L_FLOOR]) layerFlash[L_FLOOR] = now; }

  // deauth: a struck bell chord the moment a disconnect frame is heard
  uint32_t dd = deauthCount;
  if (dd != lastDeauth) {
    lastDeauth = dd;
    if (layerOn[L_DEAUTH] && now - lastBell > 4000) {         // root, fifth, octave
      float a = 0.07f * lvl(L_DEAUTH);
      schedule(now, fold(semi(chordTone(0) + 12), 196, 700), a, T_BELL, 1.0f, L_DEAUTH, -1);
      schedule(now, fold(semi(chordTone(2) + 12), 196, 1000), a * 0.8f, T_BELL, 1.0f, L_DEAUTH, -1);
      schedule(now, fold(semi(chordTone(0) + 24), 300, 1400), a * 0.6f, T_BELL, 1.0f, L_DEAUTH, -1);
      lastBell = now;
    }
  }

  // motion: a shake strums the current chord upward
  static uint32_t lastImu = 0;
  if (imuOk && now - lastImu >= 30) {
    lastImu = now;
    M5.Imu.update();
    float ax, ay, az; M5.Imu.getAccel(&ax, &ay, &az);
    float dev = fabsf(sqrtf(ax * ax + ay * ay + az * az) - 1.0f);
    shake = max(shake * 0.8f, dev);
    if (layerOn[L_MOTION] && shake > 0.35f && now - lastStrum > 700) {
      float a = (0.06f + 0.05f * min(shake, 1.5f)) * lvl(L_MOTION);
      for (int i = 0; i < 6; i++) {
        float f = fold(semi(chordTone(i % 4) + 12 + 12 * (i / 4)), 196, 1400);
        schedule(now + i * 45, f, a, T_KALIMBA, 1.2f, L_MOTION, -1);
      }
      lastStrum = now;
    }
  }

  // release everything whose step has arrived
  for (int i = 0; i < NS; i++) {
    Sched& s = sched[i];
    if (!s.used || s.at > now) continue;
    play(s.f, s.amp, s.timbre, s.len);
    layerFlash[s.layer] = now;
    if (s.voice >= 0) voices[s.voice].flash = now;
    s.used = false;
  }

  // bluetooth: scan in short windows so memory never grows
  if (layerOn[L_BLE] && !bleScanning) {
    bleScan->clearResults();
    bleScanning = true;
    bleScan->start(3, bleDone, false);
  }

  static uint32_t lastHop = 0;
  if (hopping && now - lastHop > 1500) { setChannel(channel % 13 + 1); lastHop = now; }

  static uint32_t lastDraw = 0;
  if (now - lastDraw > 60) { draw(); lastDraw = now; }
}
