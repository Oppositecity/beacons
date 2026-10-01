// BEACONS v1.3 — listening to the air, for M5Stack Cardputer (original + ADV)
//
// Nothing here is composed. Every sound is a measurement:
//   when a note sounds  = the moment a signal arrives
//   its pitch           = how close the source is: each device climbs the harmonic series as you
//                         approach it (about one step per 4-5 dB), over a fundamental set by the
//                         WiFi channel you're hearing (ch1 = 55 Hz ... ch13 = 110 Hz)
//   its bend            = whether it's getting closer or farther: approaching sources go slightly
//                         sharp, receding ones flat, like a doppler shift
//   loudness, brightness, ring length = signal strength (close = loud, bright, long)
//   timbre              = what kind of signal it is (the kit decides which instrument voices
//                         which kind; it never changes pitch, timing or loudness)
//
//   1 aps     WiFi access points announcing themselves, each sounding on its own clock
//   2 probes  phones calling for networks they remember, an octave up
//   3 data    WiFi data frames; every Nth frame sounds, pitched by its strength, ringing by its size
//   4 floor   the radio noise floor + devices present -> drone: loudness = noise floor,
//             one harmonic per device present (max 8)
//   5 deauth  disconnect frames (rare; often an attack) -> a struck bell
//   6 ble     Bluetooth devices advertising nearby, low register
//
//   kits (k)  aps       probes    data        deauth  ble
//   wood      marimba   kalimba   vibraphone  bell    tongue drum
//   metal     gamelan   bright    glass       bell    low gong
//   skin      drum      kalimba   woodblock   bell    low drum
//   glass     glass     bright    vibraphone  bell    tongue drum
//   air       sustained tones: the nearest eight devices each hold a tone at their pitch,
//             gliding as you move; each ping swells, wobbles and thickens it. Two tones near
//             the same pitch beat at a rate set by how differently they're moving.
//             data and deauth stay struck (vibraphone, bell).
//
// Pitches stay within 110–2000 Hz and the output passes a soft limiter. The speaker library's own
// 16x boost is switched off (it was clipping every struck note), so nothing past the limiter can clip.
//
// Each layer's loudness is fixed, so loudness only ever means signal strength.
//
// One screen, every mark a measurement:
//   rows    the nearest sources: up to three WiFi and three bluetooth, highest pitch (closest)
//           at top. A row flashes when it sounds; the bar is signal strength; + / - at the end
//           = approaching / receding. ? = probe, * = bluetooth.
//   below   each layer with its key and a live count: access points present, phones probing,
//           data frames per second, noise floor in dBm, disconnects heard, bluetooth devices
//           present. UPPERCASE = layer on. A label flashes when its layer sounds.
//   bottom  the controls with their current values. UPPERCASE = on.
//
// keys:  1-6 layer on/off
//        a d channel down / up     s scan all channels (back and forth)     k kit
//        f g fewer / more notes    z x volume down / up    m mute    o off (reset to wake)
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
enum { L_BEACON, L_PROBE, L_TRAFFIC, L_FLOOR, L_DEAUTH, L_BLE, NLAYERS };
enum { T_MARIMBA, T_KALIMBA, T_TONGUE, T_VIBES, T_BELL, T_GLASS, T_BRIGHT, T_GAMELAN, T_GONG, T_DRUM, T_BLOCK, T_LOWDRUM, NTIMBRES };
struct Ev { uint8_t kind; uint8_t mac[6]; int8_t rssi; char name[33]; };   // kind 0 AP, 1 probe, 2 BLE
struct Partial { float a1, a2, y1, y2, env, rblk; };
struct Note { Partial p[3]; int np; bool active; int age; };
struct NoteEv { float f, amp, len, bright; uint8_t timbre; };
struct Voice {
  uint8_t mac[6]; char s[33]; uint8_t kind; float rssi, rssiSlow;
  uint32_t seen, flash, lastSound, n; bool used;
};

// =====================================================================
//  layers and controls
// =====================================================================
static const char* LAYER_TAGS[NLAYERS] = {"aps", "probes", "data", "floor", "deauth", "ble"};
static bool     layerOn[NLAYERS]    = {true, true, true, true, true, true};
static const float CAL[NLAYERS]     = {0.7f, 0.5f, 0.5f, 0.4f, 0.6f, 0.6f};   // fixed balance between layers
static uint32_t layerFlash[NLAYERS] = {0};

static int  channel  = 6;
static bool muted    = false;
static bool hopping  = false;
static int  hopDir   = 1;
static int  kit      = 0;
static const char* KIT_NAMES[] = {"wood", "metal", "skin", "glass", "air"};
static const int AIR = 4, NKITS = 5;
// which timbre voices each layer, per kit: aps, probes, data, floor (unused), deauth, ble
static const uint8_t KIT[NKITS][NLAYERS] = {
  {T_MARIMBA, T_KALIMBA, T_VIBES, 0, T_BELL, T_TONGUE},
  {T_GAMELAN, T_BRIGHT,  T_GLASS, 0, T_BELL, T_GONG},
  {T_DRUM,    T_KALIMBA, T_BLOCK, 0, T_BELL, T_LOWDRUM},
  {T_GLASS,   T_BRIGHT,  T_VIBES, 0, T_BELL, T_TONGUE},
  {T_GLASS,   T_BRIGHT,  T_VIBES, 0, T_BELL, T_TONGUE},     // air: device layers are sustained instead
};
static int  volume   = 180;                    // with the library boost off, 255 = unity: the limiter is the ceiling
static const int VOL_MAX = 255;
static int  divIdx   = 3;
static const int DIVS[] = {1, 2, 5, 10, 20, 50};
static int  present  = 0;                      // devices heard in the last 5 s
static int  presentKind[3] = {0, 0, 0};        // access points, probing phones, bluetooth
static int  dataRate = 0;                      // data frames per second

// =====================================================================
//  radio -> loop event queue
// =====================================================================
static const int QN = 64;
static Ev q[QN];
static volatile int qHead = 0, qTail = 0;
static portMUX_TYPE qmux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t dataCount = 0, deauthCount = 0;
static volatile int noiseFloor = -95, dataRssi = -80, dataLen = 200, deauthRssi = -80;

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
  int len = p->rx_ctrl.sig_len;
  if (type == WIFI_PKT_DATA) { dataRssi = p->rx_ctrl.rssi; dataLen = len; dataCount++; return; }
  if (type != WIFI_PKT_MGMT) return;
  const uint8_t* f = p->payload;
  if (len < 24) return;
  uint8_t sub = (f[0] >> 4) & 0x0F;
  if (sub == 12 || sub == 10) { deauthRssi = p->rx_ctrl.rssi; deauthCount++; return; }   // deauth / disassoc
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
//  mapping: measurement -> sound
// =====================================================================
static float fold(float f, float lo, float hi) {
  while (f < lo) f *= 2.0f;
  while (f > hi) f *= 0.5f;
  return f;
}
static float fundamental() { return 55.0f * powf(2.0f, (channel - 1) / 12.0f); }   // one semitone per channel
static float nearness(float rssi) { return constrain((rssi + 95.0f) / 65.0f, 0.0f, 1.0f); }   // -95 dBm far .. -30 close
static int harmonic(float rssi, int lo, int hi) { return lo + (int)roundf(nearness(rssi) * (hi - lo)); }

// pitch of a device: harmonic number from its strength, bent by its approach or retreat
static float pitchFor(const Voice& v) {
  float f0 = fundamental(), f;
  if (v.kind == 2)      f = 2.0f * f0 * harmonic(v.rssi, 1, 5);      // bluetooth: low
  else if (v.kind == 1) f = 2.0f * f0 * harmonic(v.rssi, 4, 12);     // probes: an octave up
  else                  f = f0 * harmonic(v.rssi, 2, 16);            // access points: middle
  float cents = constrain((v.rssi - v.rssiSlow) * 8.0f, -40.0f, 40.0f);
  return fold(f * powf(2.0f, cents / 1200.0f), 110, 2000);
}

// =====================================================================
//  synthesis engine — modal resonators in their own task
// =====================================================================
static const int SR = 22050, BLK = 512;          // 23 ms per buffer, two queued: room to breathe
static const int NNOTES = 24;
static Note notes[NNOTES];
static const int NQ = 32;
static NoteEv nq[NQ];
static volatile int nqHead = 0, nqTail = 0;
static portMUX_TYPE nmux = portMUX_INITIALIZER_UNLOCKED;

// partial ratios, relative amplitudes, decay times (seconds to fade 60 dB)
// marimba, kalimba, tongue drum, vibraphone, bell, glass, bright glass, gamelan bar (free-bar modes),
// low gong, drum (membrane modes), woodblock, low drum
static const float T_RATIO[NTIMBRES][3] = {
  {1, 3.99f, 9.9f}, {1, 5.9f, 0}, {1, 2.0f, 3.01f}, {1, 4.0f, 0}, {1, 2.0f, 3.0f}, {1, 3.0f, 0},
  {1, 2.0f, 0}, {1, 2.76f, 5.40f}, {1, 2.76f, 0}, {1, 1.59f, 2.14f}, {1, 2.57f, 0}, {1, 1.59f, 2.14f}};
static const float T_AMP[NTIMBRES][3]   = {
  {1, 0.28f, 0.07f}, {1, 0.16f, 0}, {1, 0.33f, 0.09f}, {1, 0.12f, 0}, {1, 0.45f, 0.22f}, {1, 0.06f, 0},
  {1, 0.10f, 0}, {1, 0.45f, 0.20f}, {1, 0.30f, 0}, {1, 0.45f, 0.25f}, {1, 0.30f, 0}, {1, 0.35f, 0.15f}};
static const float T_T60[NTIMBRES][3]   = {
  {1.1f, 0.22f, 0.06f}, {1.8f, 0.25f, 0}, {1.5f, 0.5f, 0.2f}, {2.6f, 0.6f, 0}, {3.5f, 2.0f, 1.2f}, {3.2f, 0.9f, 0},
  {1.2f, 0.4f, 0}, {2.4f, 1.1f, 0.5f}, {3.0f, 1.4f, 0}, {0.4f, 0.25f, 0.15f}, {0.15f, 0.06f, 0}, {0.7f, 0.35f, 0.2f}};

// a short memory keeps identical notes from stacking and caps how many start at once
static const int NRECENT = 16;
static float recentF[NRECENT];
static uint32_t recentT[NRECENT];
static int recentI = 0;

static bool play(float f, float amp, uint8_t timbre, float len, float bright) {
  if (muted || amp <= 0) return false;
  uint32_t now = millis();
  int crowd = 0;
  for (int i = 0; i < NRECENT; i++) {
    if (now - recentT[i] > 60) continue;
    if (fabsf(recentF[i] - f) < 1.0f) return false;
    crowd++;
  }
  if (crowd >= 6) return false;
  recentF[recentI] = f; recentT[recentI] = now; recentI = (recentI + 1) % NRECENT;
  portENTER_CRITICAL(&nmux);
  int next = (nqHead + 1) % NQ;
  if (next != nqTail) { nq[nqHead] = {f, amp, len, bright, timbre}; nqHead = next; }
  portEXIT_CRITICAL(&nmux);
  return true;
}

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
    float A = e.amp * T_AMP[e.timbre][k] * (k ? e.bright : 1.0f);
    Partial& p = n.p[n.np++];
    p.a1 = 2.0f * rr * cosf(w); p.a2 = -rr * rr;
    p.y1 = A * sinf(w); p.y2 = 0;                     // starts at zero crossing: no click
    p.env = A; p.rblk = powf(rr, BLK);
  }
  n.age = 0;
  n.active = n.np > 0;
}

static const int RAMP = 88;                          // 4 ms at 22050 Hz

// air kit: up to eight sustained tones, one per nearby device
static const int NSUS = 8;
static volatile float susF[NSUS], susA[NSUS];        // target pitch and loudness, set by the loop
static volatile bool  susPing[NSUS];                 // set by the loop when that device pings
static int susVoice[NSUS] = {-1, -1, -1, -1, -1, -1, -1, -1};

// drone (floor layer): harmonics of the channel's fundamental, one per device present
static volatile float droneLevel = 0, droneF0 = 73.4f;
static volatile int dronePartials = 1;

// sine by table lookup: the sustained tones and drone call this ~700k times a second
static const int SINN = 1024;
static float SINT[SINN + 1];
static void initSin() { for (int i = 0; i <= SINN; i++) SINT[i] = sinf(2 * PI * i / SINN); }
static inline float fsin(float ph) {                 // ph in [0, 2pi)
  float x = ph * (SINN / (2 * PI));
  int i = (int)x; if (i >= SINN) i = SINN - 1;
  float fr = x - i;
  return SINT[i] + (SINT[i + 1] - SINT[i]) * fr;
}

static float softclip(float x) {
  if (x > 3) x = 3; else if (x < -3) x = -3;
  return x * (27 + x * x) / (27 + 9 * x * x);
}

static int16_t outbuf[3][BLK];
static float mix[BLK];

static void synthTask(void*) {
  int ob = 0;
  float ph[8] = {0}, g[8] = {0}, f0s = 73.4f, lfo = 0;
  float sf[NSUS], sa[NSUS] = {0}, sp[NSUS] = {0}, sph[NSUS] = {0}, sph2[NSUS] = {0}, svph[NSUS] = {0};
  for (int i = 0; i < NSUS; i++) sf[i] = 220;
  const float pingDecay = expf(-1.0f / (0.3f * SR));   // a ping fades over about a third of a second
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
        int age = n.age;
        for (int s = 0; s < BLK; s++) {
          float y = a1 * y1 + a2 * y2;
          y2 = y1; y1 = y;
          int a = age + s;
          mix[s] += (a < RAMP) ? y * a / (float)RAMP : y;   // 4 ms fade-in: a strike, not a click
        }
        p.y1 = y1; p.y2 = y2; p.env *= p.rblk;
        if (p.env > 1e-4f) alive = true;
      }
      n.age += BLK;
      n.active = alive;
    }

    // drone: partial k+1 fades in when at least k+1 devices are present
    float lv = droneLevel, ft = droneF0; int nk = dronePartials;
    float tg[8];
    for (int k = 0; k < 8; k++) tg[k] = (k < nk) ? lv / (1.0f + 0.6f * k) : 0.0f;
    for (int s = 0; s < BLK; s++) {
      f0s += (ft - f0s) * 0.0002f;
      lfo += 2 * PI * 0.07f / SR; if (lfo > 2 * PI) lfo -= 2 * PI;
      float breath = 0.75f + 0.25f * fsin(lfo), acc = 0;
      for (int k = 0; k < 8; k++) {
        g[k] += (tg[k] - g[k]) * 0.0002f;
        float fk = f0s * 2.0f * (k + 1);                // start an octave above the fundamental
        if (fk > 2000) continue;
        ph[k] += 2 * PI * fk / SR; if (ph[k] > 2 * PI) ph[k] -= 2 * PI;
        if (g[k] > 1e-5f) acc += g[k] * fsin(ph[k]);
      }
      mix[s] += breath * acc;
    }

    // air kit: sustained tones glide to their device's pitch; a ping swells, wobbles and thickens
    for (int i = 0; i < NSUS; i++) {
      if (susPing[i]) { susPing[i] = false; sp[i] = 1.0f; }
      float ft = susF[i], at = susA[i];
      if (at < 1e-5f && sa[i] < 1e-5f) { sa[i] = 0; sf[i] = ft; continue; }
      for (int s = 0; s < BLK; s++) {
        sf[i] += (ft - sf[i]) * 0.0004f;
        sa[i] += (at - sa[i]) * 0.0003f;
        sp[i] *= pingDecay;
        svph[i] += 2 * PI * 5.5f / SR; if (svph[i] > 2 * PI) svph[i] -= 2 * PI;
        float inc = 2 * PI * sf[i] * (1.0f + 0.012f * sp[i] * fsin(svph[i])) / SR;
        sph[i] += inc;      if (sph[i] > 2 * PI) sph[i] -= 2 * PI;
        sph2[i] += 2 * inc; if (sph2[i] > 2 * PI) sph2[i] -= 2 * PI;
        mix[s] += sa[i] * (1.0f + 0.7f * sp[i]) * (fsin(sph[i]) + (0.12f + 0.5f * sp[i]) * fsin(sph2[i]));
      }
    }

    int16_t* o = outbuf[ob];
    for (int s = 0; s < BLK; s++) o[s] = (int16_t)(softclip(mix[s] * 1.8f) * 21000.0f);
    M5Cardputer.Speaker.playRaw(o, BLK, SR, false, 1, 0, false);
    ob = (ob + 1) % 3;
  }
}

// =====================================================================
//  sources on screen: one per device heard
// =====================================================================
static const int NV = 48;
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
    memcpy(v.mac, e.mac, 6); v.kind = e.kind; v.rssi = v.rssiSlow = e.rssi;
    v.n = esp_random() % 50; v.flash = 0; v.lastSound = 0; v.used = true; v.s[0] = 0;
  }
  char s[33]; strcpy(s, e.name);
  if (printable(s)) strcpy(v.s, s);
  else if (!v.s[0]) {
    if (e.kind == 1) strcpy(v.s, "any");
    else snprintf(v.s, sizeof(v.s), "%s %02x%02x", e.kind == 2 ? "ble" : "hidden", e.mac[4], e.mac[5]);
  }
  v.rssi = v.rssi * 0.6f + e.rssi * 0.4f;              // quick: follows you as you move
  v.rssiSlow = v.rssiSlow * 0.95f + e.rssi * 0.05f;    // slow: what it was a few seconds ago
  v.seen = millis();
  return slot;
}

static void setChannel(int ch) {
  channel = ch;
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  for (int i = 0; i < NV; i++) if (voices[i].kind != 2) voices[i].used = false;
}

static float lvl(int L) { return layerOn[L] ? CAL[L] : 0.0f; }

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
  for (char c : ks.word) {
    if (c >= '1' && c <= '6') layerOn[c - '1'] = !layerOn[c - '1'];
    else if (c == 'a') setChannel((channel + 11) % 13 + 1);
    else if (c == 'd') setChannel(channel % 13 + 1);
    else if (c == 's') hopping = !hopping;
    else if (c == 'f') divIdx = min(divIdx + 1, 5);
    else if (c == 'g') divIdx = max(divIdx - 1, 0);
    else if (c == 'z') { volume = max(volume - 25, 0);       M5Cardputer.Speaker.setVolume(volume); }
    else if (c == 'x') { volume = min(volume + 25, VOL_MAX); M5Cardputer.Speaker.setVolume(volume); }
    else if (c == 'm') muted = !muted;
    else if (c == 'k') kit = (kit + 1) % NKITS;
    else if (c == 'o') goToSleep();
  }
}

// =====================================================================
//  screen
// =====================================================================
// strongest alive source of one family (wifi: kinds 0 and 1, or bluetooth: kind 2) not yet taken
static int strongest(uint32_t now, bool ble, bool* taken) {
  int best = -1;
  for (int i = 0; i < NV; i++) {
    Voice& v = voices[i];
    if (!v.used || taken[i] || now - v.seen > 5000 || (v.kind == 2) != ble) continue;
    int L = v.kind == 0 ? L_BEACON : (v.kind == 1 ? L_PROBE : L_BLE);
    if (!layerOn[L]) continue;
    if (best < 0 || v.rssi > voices[best].rssi) best = i;
  }
  return best;
}

// one layer label: key, name (UPPERCASE = on), live count; inverted while that layer sounds
static int drawTag(int x, int y, int L, const char* count, uint32_t now) {
  char name[12], t[32];
  strcpy(name, LAYER_TAGS[L]);
  if (layerOn[L]) for (char* c = name; *c; c++) *c = toupper(*c);
  snprintf(t, sizeof(t), "%d %s %s", L + 1, name, count);
  int w = strlen(t) * 6;
  bool lit = layerOn[L] && now - layerFlash[L] < 140;
  if (lit) canvas.fillRect(x - 1, y - 1, w + 2, 9, TFT_WHITE);
  canvas.setTextColor(lit ? TFT_BLACK : TFT_WHITE);
  canvas.drawString(t, x, y);
  return x + w + 12;
}

static void draw() {
  canvas.fillSprite(TFT_BLACK);
  uint32_t now = millis();
  const int ROWS = 5, ROW_H = 16, BAR_X = 150, BAR_W = 76;

  // rows: up to three of each family, so neither can crowd the other out
  int pick[ROWS], np = 0;
  bool taken[NV] = {false};
  for (int fam = 0; fam < 2; fam++)
    for (int r = 0; r < 3 && np < ROWS; r++) {
      int b = strongest(now, fam == 1, taken);
      if (b < 0) break;
      taken[b] = true; pick[np++] = b;
    }
  while (np < ROWS) {                                   // fill leftover rows from whichever family has more
    int b0 = strongest(now, false, taken), b1 = strongest(now, true, taken);
    int b = b0 < 0 ? b1 : (b1 < 0 ? b0 : (voices[b0].rssi >= voices[b1].rssi ? b0 : b1));
    if (b < 0) break;
    taken[b] = true; pick[np++] = b;
  }
  for (int a = 0; a < np; a++)
    for (int b = a + 1; b < np; b++)
      if (pitchFor(voices[pick[b]]) > pitchFor(voices[pick[a]])) { int t = pick[a]; pick[a] = pick[b]; pick[b] = t; }

  canvas.setFont(&fonts::FreeMono9pt7b);
  for (int r = 0; r < np; r++) {
    Voice& v = voices[pick[r]];
    int y = 1 + r * ROW_H;
    bool lit = (now - v.flash < 140);
    uint16_t fg = lit ? TFT_BLACK : TFT_WHITE;
    if (lit) canvas.fillRect(0, y - 1, 240, ROW_H - 1, TFT_WHITE);
    canvas.setTextColor(fg);
    char label[14];
    snprintf(label, sizeof(label), "%s%.12s", v.kind == 1 ? "?" : (v.kind == 2 ? "*" : ""), v.s);
    canvas.drawString(label, 4, y);
    int w = 2 + (int)(nearness(v.rssi) * BAR_W);
    canvas.fillRect(BAR_X, y + 5, w, 4, fg);
    float trend = v.rssi - v.rssiSlow;
    if (fabsf(trend) > 1.5f) canvas.drawString(trend > 0 ? "+" : "-", 229, y);
  }

  // layers with live counts
  canvas.setFont(&fonts::Font0);
  char c[6][12];
  snprintf(c[0], 12, "%d", presentKind[0]);
  snprintf(c[1], 12, "%d", presentKind[1]);
  snprintf(c[2], 12, "%d/s", dataRate);
  snprintf(c[3], 12, "%d", noiseFloor);
  snprintf(c[4], 12, "%u", (unsigned)deauthCount);
  snprintf(c[5], 12, "%d", presentKind[2]);
  int x = 4;
  for (int L = 0; L < 3; L++) x = drawTag(x, 86, L, c[L], now);
  x = 4;
  for (int L = 3; L < 6; L++) x = drawTag(x, 97, L, c[L], now);

  canvas.setTextColor(TFT_WHITE);
  char l3[48], l4[48];
  snprintf(l3, sizeof(l3), "a d ch%d %dMHz  s %s  k %s",
           channel, 2407 + 5 * channel, hopping ? "SCAN" : "scan", KIT_NAMES[kit]);
  snprintf(l4, sizeof(l4), "f g /%d  z x vol%d  m %s  o off",
           DIVS[divIdx], volume, muted ? "MUTED" : "mute");
  canvas.drawString(l3, 4, 112);
  canvas.drawString(l4, 4, 123);
  canvas.pushSprite(0, 0);
}

// =====================================================================
//  setup / loop
// =====================================================================
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  // the library multiplies output 16x by default, tuned for quiet beeps; our signal is already full
  // strength, so that boost clipped every note. At 1x, our soft limiter is the only ceiling.
  M5Cardputer.Speaker.end();
  auto spk = M5Cardputer.Speaker.config();
  spk.magnification = 1;
  M5Cardputer.Speaker.config(spk);
  M5Cardputer.Speaker.begin();
  M5Cardputer.Speaker.setVolume(volume);
  canvas.createSprite(240, 135);
  initSin();

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

  // devices: every note sounds the moment its signal arrives
  Ev e;
  int budget = 32;
  while (budget-- && popEv(e)) {
    int L = e.kind == 0 ? L_BEACON : (e.kind == 1 ? L_PROBE : L_BLE);
    if (!layerOn[L]) continue;
    int vi = hear(e);
    Voice& v = voices[vi];
    float nr = nearness(v.rssi);
    float amp = 0.04f + 0.24f * nr, bright = 0.3f + 1.3f * nr, len = 0.5f + 1.5f * nr;
    bool sounded = false;
    if (kit == AIR) {                                         // air: a ping swells that device's tone
      if (e.kind == 0 && (++v.n % DIVS[divIdx])) continue;
      if (e.kind != 0 && now - v.lastSound < (e.kind == 1 ? 400u : 60u * DIVS[divIdx])) continue;
      for (int i = 0; i < NSUS; i++) if (susVoice[i] == vi) { susPing[i] = true; sounded = true; }
    } else if (e.kind == 0) {
      if (++v.n % DIVS[divIdx]) continue;                    // each AP on its own real clock, thinned
      sounded = play(pitchFor(v), amp * lvl(L_BEACON), KIT[kit][L_BEACON], len, bright);
    } else if (e.kind == 1) {
      if (now - v.lastSound < 400) continue;
      sounded = play(pitchFor(v), amp * 0.7f * lvl(L_PROBE), KIT[kit][L_PROBE], len, bright);
    } else {
      if (now - v.lastSound < 60u * DIVS[divIdx]) continue;
      sounded = play(pitchFor(v), amp * lvl(L_BLE), KIT[kit][L_BLE], len, bright);
    }
    v.lastSound = now;
    if (sounded) { v.flash = now; layerFlash[L] = now; }
  }

  // traffic: every Nth data frame sounds, pitched by that frame's strength, ringing by its size
  static uint32_t lastData = 0, lastTraffic = 0, lastDeauth = 0, lastBell = 0;
  uint32_t dc = dataCount, framesPerNote = 4 * DIVS[divIdx];
  if (dc - lastData >= framesPerNote) {
    lastData = dc - (dc - lastData) % framesPerNote;
    if (layerOn[L_TRAFFIC] && now - lastTraffic >= 70) {
      float r = dataRssi, nr = nearness(r);
      float f = fold(fundamental() * harmonic(r, 3, 16), 110, 2000);
      float len = 0.4f + 2.0f * constrain(dataLen / 1500.0f, 0.0f, 1.0f);
      if (play(f, (0.03f + 0.14f * nr) * lvl(L_TRAFFIC), KIT[kit][L_TRAFFIC], len, 0.3f + 1.2f * nr)) layerFlash[L_TRAFFIC] = now;
      lastTraffic = now;
    }
  }

  // deauth: a struck bell the moment a disconnect frame is heard, pitched by its strength
  uint32_t dd = deauthCount;
  if (dd != lastDeauth) {
    lastDeauth = dd;
    if (layerOn[L_DEAUTH] && now - lastBell > 2500) {
      float r = deauthRssi, nr = nearness(r);
      float f = fold(fundamental() * 2.0f * harmonic(r, 2, 8), 110, 1400);
      if (play(f, (0.05f + 0.1f * nr) * lvl(L_DEAUTH), KIT[kit][L_DEAUTH], 1.0f, 0.5f + nr)) layerFlash[L_DEAUTH] = now;
      lastBell = now;
    }
  }

  // air kit: give the nearest eight devices a sustained tone; keep a tone with its device while it stays near
  static uint32_t lastSus = 0;
  if (now - lastSus > 100) {
    lastSus = now;
    bool want[NV] = {false};
    if (kit == AIR && !muted)
      for (int r = 0; r < NSUS; r++) {
        int best = -1;
        for (int i = 0; i < NV; i++) {
          Voice& v = voices[i];
          if (!v.used || want[i] || now - v.seen > 5000) continue;
          int L = v.kind == 0 ? L_BEACON : (v.kind == 1 ? L_PROBE : L_BLE);
          if (!layerOn[L]) continue;
          if (best < 0 || v.rssi > voices[best].rssi) best = i;
        }
        if (best < 0) break;
        want[best] = true;
      }
    for (int i = 0; i < NSUS; i++) if (susVoice[i] >= 0 && !want[susVoice[i]]) susVoice[i] = -1;   // let it fade
    for (int vi = 0; vi < NV; vi++) {
      if (!want[vi]) continue;
      int slot = -1;
      for (int i = 0; i < NSUS; i++) if (susVoice[i] == vi) slot = i;
      if (slot < 0) for (int i = 0; i < NSUS; i++) if (susVoice[i] < 0 && susA[i] < 1e-5f) { slot = i; break; }
      if (slot < 0) for (int i = 0; i < NSUS; i++) if (susVoice[i] < 0) { slot = i; break; }
      if (slot >= 0) susVoice[slot] = vi;
    }
    for (int i = 0; i < NSUS; i++) {
      if (susVoice[i] < 0) { susA[i] = 0; continue; }
      Voice& v = voices[susVoice[i]];
      int L = v.kind == 0 ? L_BEACON : (v.kind == 1 ? L_PROBE : L_BLE);
      float nr = nearness(v.rssi);
      susF[i] = pitchFor(v);
      susA[i] = (0.006f + 0.06f * nr * nr) * lvl(L);
    }
  }

  // floor: drone loudness = noise floor; one harmonic per device present
  static uint32_t lastCount = 0;
  if (now - lastCount > 250) {
    lastCount = now;
    int n = 0, k[3] = {0, 0, 0};
    for (int i = 0; i < NV; i++) if (voices[i].used && now - voices[i].seen < 5000) { n++; k[voices[i].kind]++; }
    present = n;
    for (int j = 0; j < 3; j++) presentKind[j] = k[j];
    static uint32_t lastRateCount = 0;
    uint32_t dcNow = dataCount;
    dataRate = (dataRate + (int)((dcNow - lastRateCount) * 4)) / 2;   // frames per second, lightly smoothed
    lastRateCount = dcNow;
  }
  float nf = constrain((noiseFloor + 100) / 20.0f, 0.0f, 1.0f);
  droneF0 = fundamental();
  dronePartials = constrain(present, 1, 8);
  droneLevel = muted ? 0 : (0.015f + 0.04f * nf) * lvl(L_FLOOR);
  static int lastNf = -95;
  if (abs(noiseFloor - lastNf) >= 2) { lastNf = noiseFloor; if (layerOn[L_FLOOR]) layerFlash[L_FLOOR] = now; }

  // bluetooth: scan in short windows so memory never grows
  if (layerOn[L_BLE] && !bleScanning) {
    bleScan->clearResults();
    bleScanning = true;
    bleScan->start(3, bleDone, false);
  }

  static uint32_t lastHop = 0;
  if (hopping && now - lastHop > 1500) {                  // back and forth, so the sweep never jumps
    if (channel + hopDir > 13 || channel + hopDir < 1) hopDir = -hopDir;
    setChannel(channel + hopDir);
    lastHop = now;
  }

  static uint32_t lastDraw = 0;
  if (now - lastDraw > 60) { draw(); lastDraw = now; }
}
