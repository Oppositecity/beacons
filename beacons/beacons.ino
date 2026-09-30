// BEACONS v0.6 — the air around you as a small orchestra, for M5Stack Cardputer (original + ADV)
//
// Seven layers, each a different signal heard by the radio (or felt by the body):
//   1 beacons    WiFi access points announcing themselves      -> marimba
//   2 probes     phones calling for networks they remember     -> kalimba
//   3 traffic    WiFi data volume on this channel              -> rain
//   4 floor      the radio noise floor, the air's own hiss     -> low drone, like surf
//   5 deauth     disconnect frames (rare; often an attack)     -> birdsong
//   6 bluetooth  BLE devices advertising nearby                -> tongue drum
//   7 motion     shaking the Cardputer (ADV motion sensor)     -> rainstick
//
// All synthesis is modal: struck-bar physics, sines with natural decays, soft attacks.
// Every pitch is folded into 130–2600 Hz, the output passes a soft limiter, and
// volume is capped, so nothing clicks, clips, or reaches the extremes of hearing.
//
// Two screens, switched with tab:
//   play   up to six voices as rows, highest pitch at top; a row flashes when it sounds,
//          the bar is signal strength. ? = probe, * = bluetooth.
//   mixer  the seven layers; UPPERCASE = on, the bar is level, a row flashes when that layer sounds.
//
// keys (both screens):  1-7 layer on/off   tab switch screen   m mute   q sleep (reset to wake)
// play screen:          , / channel   h hop   ; . octave   s scale   l length   [ ] thin/thicken   - = volume
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
//  layers
// =====================================================================
enum { L_BEACON, L_PROBE, L_RAIN, L_FLOOR, L_DEAUTH, L_BLE, L_MOTION, NLAYERS };
static const char* LAYER_NAMES[NLAYERS] = {"beacons", "probes", "traffic", "floor", "deauth", "bluetooth", "motion"};
static bool     layerOn[NLAYERS]    = {true, true, true, true, true, true, true};
static int      layerLvl[NLAYERS]   = {7, 5, 4, 3, 6, 6, 6};     // 0..10
static uint32_t layerFlash[NLAYERS] = {0};
static bool     imuOk = false;

// =====================================================================
//  controls
// =====================================================================
static int  channel  = 6;
static bool muted    = false;
static bool hopping  = false;
static int  octave   = 0;
static int  scaleIdx = 0;
static int  lenIdx   = 1;
static int  volume   = 120;
static const int VOL_MAX = 200;                // hard cap for ears and equipment
static int  divIdx   = 3;
static bool mixerView = false;
static int  sel      = 0;
static const float LENS[] = {0.5f, 1.0f, 2.0f, 4.0f};
static const char* SCALE_NAMES[] = {"penta", "harm", "whole", "chrom"};
static const int DIVS[] = {1, 2, 5, 10, 20, 50};

// =====================================================================
//  radio -> loop event queue
// =====================================================================
struct Ev { uint8_t kind; uint8_t mac[6]; int8_t rssi; char name[33]; };   // kind 0 AP, 1 probe, 2 BLE
// every type used in a function signature is defined up here, before the first function,
// so the Arduino builder's auto-generated prototypes always compile
enum { T_MARIMBA, T_KALIMBA, T_TONGUE, T_DROP };
struct Partial { float a1, a2, y1, y2, env, rblk; };
struct Note { Partial p[3]; int np; bool active; };
struct NoteEv { float f, amp, len; uint8_t timbre; };
struct Voice {
  uint8_t mac[6]; char s[33]; uint8_t kind; int rssi;
  uint32_t seen, flash, lastSound, n; bool used;
};
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
//  pitch
// =====================================================================
static uint32_t hashMac(const uint8_t* m) {
  uint32_t h = 2166136261u;
  for (int i = 0; i < 6; i++) { h ^= m[i]; h *= 16777619u; }
  return h;
}
static float fold(float f, float lo, float hi) {   // keep every pitch in a gentle band
  while (f < lo) f *= 2.0f;
  while (f > hi) f *= 0.5f;
  return f;
}
static float scaleNote(uint32_t h, int octBase) {
  float base = 110.0f * powf(2.0f, (octBase + octave));
  if (scaleIdx == 1) return base * (float)(1 + (h % 16));
  int semis;
  if (scaleIdx == 0) { static const int P[] = {0, 2, 4, 7, 9}; semis = P[h % 5]; }
  else if (scaleIdx == 2) semis = 2 * (int)(h % 6);
  else semis = (int)(h % 12);
  semis += 12 * (int)((h >> 8) % 3);
  return base * powf(2.0f, semis / 12.0f);
}
static float pitchFor(const uint8_t* mac, int kind) {
  int ob = kind == 1 ? 3 : (kind == 2 ? 0 : 1);
  float lo = kind == 2 ? 130.0f : 196.0f, hi = kind == 1 ? 2000.0f : 1400.0f;
  return fold(scaleNote(hashMac(mac), ob), lo, hi);
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
static const float T_RATIO[4][3] = {{1, 3.99f, 9.9f}, {1, 5.9f, 0}, {1, 2.0f, 3.01f}, {1, 2.4f, 0}};
static const float T_AMP[4][3]   = {{1, 0.28f, 0.07f}, {1, 0.16f, 0}, {1, 0.33f, 0.09f}, {1, 0.12f, 0}};
static const float T_T60[4][3]   = {{1.1f, 0.22f, 0.06f}, {1.8f, 0.25f, 0}, {1.5f, 0.5f, 0.2f}, {0.14f, 0.05f, 0}};

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

// drone (floor layer): root + fifth + octave, slow detune like surf
static volatile float droneTarget = 0, droneRoot = 220.0f;
// birdsong (deauth layer)
static volatile bool birdTrigger = false;
static volatile float birdAmp = 0;

static float softclip(float x) {
  if (x > 3) x = 3; else if (x < -3) x = -3;
  return x * (27 + x * x) / (27 + 9 * x * x);
}

static int16_t outbuf[3][BLK];
static float mix[BLK];

static void synthTask(void*) {
  int ob = 0;
  float d1 = 0, d2 = 0, d3 = 0, lfo = 0, droneGain = 0;
  float bPh = 0; int bPos = -1, bLen = 0, bReps = 0, bGap = 0; float bF0 = 0, bF1 = 0;
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

    // drone
    float dt = droneTarget, root = droneRoot;
    float w1 = 2 * PI * root / SR, w2 = 2 * PI * (root * 1.5f + 0.35f) / SR, w3 = 2 * PI * (root * 2.0f - 0.2f) / SR;
    for (int s = 0; s < BLK; s++) {
      droneGain += (dt - droneGain) * 0.0004f;
      if (droneGain > 1e-5f) {
        lfo += 2 * PI * 0.07f / SR; if (lfo > 2 * PI) lfo -= 2 * PI;
        float breath = 0.7f + 0.3f * sinf(lfo);
        mix[s] += droneGain * breath * (sinf(d1) + 0.55f * sinf(d2) + 0.22f * sinf(d3));
      }
      d1 += w1; d2 += w2; d3 += w3;
      if (d1 > 2 * PI) d1 -= 2 * PI; if (d2 > 2 * PI) d2 -= 2 * PI; if (d3 > 2 * PI) d3 -= 2 * PI;
    }

    // birdsong: two or three rising chirps with a little vibrato
    if (birdTrigger && bPos < 0) {
      birdTrigger = false;
      bReps = 2 + (esp_random() % 2); bLen = SR * 0.11f; bGap = SR * 0.07f; bPos = 0;
      bF0 = 1700 + (esp_random() % 300); bF1 = bF0 * 1.35f;
    }
    if (bPos >= 0) {
      for (int s = 0; s < BLK; s++) {
        if (bPos < bLen) {
          float t = (float)bPos / bLen;
          float f = bF0 + (bF1 - bF0) * t + 40 * sinf(2 * PI * 28 * bPos / SR);
          float env = sinf(PI * t); env *= env;
          bPh += 2 * PI * f / SR; if (bPh > 2 * PI) bPh -= 2 * PI;
          mix[s] += birdAmp * env * sinf(bPh);
        }
        if (++bPos >= bLen + bGap) {
          if (--bReps > 0) { bPos = 0; bF0 *= 1.06f; bF1 *= 1.06f; }
          else { bPos = -1; break; }
        }
      }
    }

    int16_t* o = outbuf[ob];
    for (int s = 0; s < BLK; s++) o[s] = (int16_t)(softclip(mix[s] * 1.8f) * 21000.0f);
    M5Cardputer.Speaker.playRaw(o, BLK, SR, false, 1, 0, false);
    ob = (ob + 1) % 3;
  }
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

static Voice& hear(const Ev& e) {
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
  return v;
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
      else if (c == ';') octave = min(octave + 1, 2);
      else if (c == '.') octave = max(octave - 1, -2);
      else if (c == 's') scaleIdx = (scaleIdx + 1) % 4;
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
  snprintf(l1, sizeof(l1), ",/ ch%-2d   h %s   ;. oct%+d", channel, hopping ? "HOP" : "hop", octave);
  snprintf(l2, sizeof(l2), "s %-5s  l x%g  [] /%d  -= vol%d", SCALE_NAMES[scaleIdx], LENS[lenIdx], DIVS[divIdx], volume);
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

  // radio events -> notes
  Ev e;
  int budget = 32;
  while (budget-- && popEv(e)) {
    int L = e.kind == 0 ? L_BEACON : (e.kind == 1 ? L_PROBE : L_BLE);
    if (!layerOn[L]) continue;
    Voice& v = hear(e);
    float near = (constrain((int)e.rssi, -90, -30) + 90) / 60.0f;   // 0 far .. 1 close
    float f = pitchFor(v.mac, v.kind);
    if (e.kind == 0) {
      if (++v.n % DIVS[divIdx]) continue;                  // each AP on its own real clock, thinned
      play(f, (0.05f + 0.22f * near) * lvl(L_BEACON), T_MARIMBA, LENS[lenIdx]);
    } else if (e.kind == 1) {
      if (now - v.lastSound < 400) continue;
      play(f, 0.13f * lvl(L_PROBE), T_KALIMBA, LENS[lenIdx]);
    } else {
      if (now - v.lastSound < 100u * DIVS[divIdx]) continue;
      play(f, (0.04f + 0.18f * near) * lvl(L_BLE), T_TONGUE, LENS[lenIdx]);
    }
    v.lastSound = now; v.flash = now; layerFlash[L] = now;
  }

  // every 50 ms: rain, floor drone, deauth birds, motion rainstick
  static uint32_t lastSlow = 0, lastData = 0, lastDeauth = 0, lastBird = 0;
  static float rainRate = 0, shake = 0;
  if (now - lastSlow >= 50) {
    lastSlow = now;
    uint32_t dc = dataCount; float perSec = (dc - lastData) * 20.0f; lastData = dc;
    rainRate += (perSec - rainRate) * 0.2f;
    float dropsPerTick = min(18.0f, rainRate / 45.0f) * 0.05f * lvl(L_RAIN) * 1.5f;
    while (dropsPerTick > 0) {
      if ((esp_random() % 1000) < dropsPerTick * 1000) {
        float df = fold(scaleNote(esp_random(), 3), 900, 2600);
        play(df, (0.02f + 0.03f * (esp_random() % 100) / 100.0f) * lvl(L_RAIN) * 1.6f, T_DROP, 1.0f);
        layerFlash[L_RAIN] = now;
      }
      dropsPerTick -= 1.0f;
    }

    float nf = constrain((noiseFloor + 97) / 15.0f, 0.0f, 1.0f);
    droneRoot = fold(110.0f * powf(2.0f, octave + 1), 130, 330);
    droneTarget = muted ? 0 : (0.6f + 0.4f * nf) * 0.05f * lvl(L_FLOOR);
    static int lastNf = -95;                            // the floor row flashes when the noise floor shifts
    if (abs(noiseFloor - lastNf) >= 2) { lastNf = noiseFloor; if (layerOn[L_FLOOR]) layerFlash[L_FLOOR] = now; }

    uint32_t dd = deauthCount;
    if (dd != lastDeauth) {
      lastDeauth = dd;
      if (layerOn[L_DEAUTH] && !muted && now - lastBird > 1500) {
        birdAmp = 0.06f * lvl(L_DEAUTH); birdTrigger = true; lastBird = now; layerFlash[L_DEAUTH] = now;
      }
    }

    if (imuOk) {
      M5.Imu.update();
      float ax, ay, az; M5.Imu.getAccel(&ax, &ay, &az);
      float dev = fabsf(sqrtf(ax * ax + ay * ay + az * az) - 1.0f);
      shake = max(shake * 0.85f, dev);
      if (layerOn[L_MOTION] && shake > 0.25f) {
        int n = min(4, (int)(shake * 3));
        for (int i = 0; i < n; i++) {
          if (esp_random() % 3) continue;
          float df = fold(scaleNote(esp_random(), 3), 700, 2400);
          play(df, (0.03f + 0.04f * min(shake, 1.5f)) * lvl(L_MOTION), T_DROP, 1.4f);
        }
        layerFlash[L_MOTION] = now;
      }
    }
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
