// BEACONS v0.5 — listening instrument for M5Stack Cardputer (original + ADV)
// Every access point on the current channel becomes a voice with a fixed pitch.
// Beacons (~10/sec per AP) are thinned so each AP pulses on its own real clock;
// closer = louder. Probe requests (phones calling for remembered networks)
// sound an octave band higher and are marked with ?.
//
// Screen: up to six voices as rows, highest pitch at top. A row flashes
// inverted when it sounds; the bar is signal strength. The three lines at the
// bottom are the controls, each key next to its current value. UPPERCASE = on.
//
// keys:  , /  channel down / up        h  hop through all channels
//        ; .  octave up / down         s  scale
//        l    note length              [ ]  thin / thicken
//        - =  volume                   v  voice: pluck / click
//        p    probes on / off          m  mute
//        q    sleep (reset to wake)
// Nothing is stored or transmitted. Listening only.

#include <M5Cardputer.h>
#include <WiFi.h>
#include "esp_wifi.h"

// ---------- event queue (wifi task -> loop) ----------
struct Ev { uint8_t kind; uint8_t mac[6]; int8_t rssi; char ssid[33]; };
static const int QN = 64;
static Ev q[QN];
static volatile int qHead = 0, qTail = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ---------- controls ----------
static int  channel    = 6;
static bool showProbes = true;
static bool muted      = false;
static bool hopping    = false;
static int  octave     = 0;
static int  scaleIdx   = 0;
static int  lenIdx     = 1;
static int  volume     = 140;
static int  divIdx     = 3;
static bool pluck      = true;
static const float LENS[] = {0.5f, 1.0f, 2.0f, 4.0f};
static const char* SCALE_NAMES[] = {"penta", "harm", "whole", "chrom"};
static const int DIVS[] = {1, 2, 5, 10, 20, 50};

// ---------- voices: one per device heard ----------
struct Voice {
  uint8_t mac[6]; char s[33]; bool probe; int rssi;
  uint32_t seen, flash, n; bool used;
};
static const int NV = 24;
static Voice voices[NV];

M5Canvas canvas(&M5Cardputer.Display);

static uint32_t hashMac(const uint8_t* m) {
  uint32_t h = 2166136261u;
  for (int i = 0; i < 6; i++) { h ^= m[i]; h *= 16777619u; }
  return h;
}

static float pitchFor(const uint8_t* mac, int octBase) {
  uint32_t h = hashMac(mac);
  float base = 110.0f * powf(2.0f, (octBase + octave));
  if (scaleIdx == 1) return base * (float)(1 + (h % 16));   // harmonic series 1..16
  int semis;
  if (scaleIdx == 0) { static const int P[] = {0, 2, 4, 7, 9}; semis = P[h % 5]; }
  else if (scaleIdx == 2) semis = 2 * (int)(h % 6);         // whole-tone
  else semis = (int)(h % 12);                               // chromatic
  semis += 12 * (int)((h >> 8) % 3);
  return base * powf(2.0f, semis / 12.0f);
}
static float voicePitch(const Voice& v) { return pitchFor(v.mac, v.probe ? 3 : 0); }

// ---------- pluck synthesis: sine with exponential decay ----------
static const int POOL = 6, VLEN = 8000, VRATE = 16000;
static int16_t vbuf[POOL][VLEN];
static int nextBuf = 0;
static void playPluck(float freq, int amp, int len) {
  int b = nextBuf; nextBuf = (nextBuf + 1) % POOL;
  M5Cardputer.Speaker.stop(b);
  if (len > VLEN) len = VLEN;
  float ph = 0, dph = 2.0f * PI * freq / VRATE, env = 1.0f, k = expf(-5.0f / len);
  for (int i = 0; i < len; i++) {
    float a = (i < 40) ? env * i / 40.0f : env;
    vbuf[b][i] = (int16_t)(amp * a * sinf(ph));
    ph += dph; if (ph > 2 * PI) ph -= 2 * PI;
    env *= k;
  }
  M5Cardputer.Speaker.playRaw(vbuf[b], len, VRATE, false, 1, b, true);
}

// ---------- promiscuous callback (wifi task: keep it tiny) ----------
static void onPkt(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  auto* p = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* f = p->payload;
  int len = p->rx_ctrl.sig_len;
  if (len < 24) return;
  uint8_t sub = (f[0] >> 4) & 0x0F;
  uint8_t kind; int tag;
  if (sub == 8)      { kind = 0; tag = 36; }   // beacon
  else if (sub == 4) { kind = 1; tag = 24; }   // probe request
  else return;
  Ev e;
  e.kind = kind;
  e.rssi = p->rx_ctrl.rssi;
  memcpy(e.mac, f + 10, 6);
  e.ssid[0] = 0;
  if (len > tag + 2 && f[tag] == 0) {
    int sl = f[tag + 1]; if (sl > 32) sl = 32;
    if (tag + 2 + sl <= len) { memcpy(e.ssid, f + tag + 2, sl); e.ssid[sl] = 0; }
  }
  portENTER_CRITICAL(&mux);
  int next = (qHead + 1) % QN;
  if (next != qTail) { q[qHead] = e; qHead = next; }
  portEXIT_CRITICAL(&mux);
}

static bool pop(Ev& e) {
  bool ok = false;
  portENTER_CRITICAL(&mux);
  if (qTail != qHead) { e = q[qTail]; qTail = (qTail + 1) % QN; ok = true; }
  portEXIT_CRITICAL(&mux);
  return ok;
}

static bool printable(char* s) {
  bool any = false;
  for (char* c = s; *c; c++) {
    if (*c < 32 || *c > 126) *c = '.';
    else if (*c != ' ') any = true;
  }
  return any;
}

// find or claim the voice for this device, update what we know about it
static Voice& hear(const Ev& e) {
  bool pr = (e.kind == 1);
  int slot = -1, oldest = 0;
  for (int i = 0; i < NV; i++) {
    if (voices[i].used && voices[i].probe == pr && memcmp(voices[i].mac, e.mac, 6) == 0) { slot = i; break; }
    if (!voices[i].used) { if (slot < 0) slot = i; }
    else if (voices[i].seen < voices[oldest].seen) oldest = i;
  }
  if (slot < 0) slot = oldest;
  Voice& v = voices[slot];
  if (!v.used || memcmp(v.mac, e.mac, 6) != 0 || v.probe != pr) {
    memcpy(v.mac, e.mac, 6); v.probe = pr; v.rssi = e.rssi;
    v.n = esp_random() % 50; v.flash = 0; v.used = true; v.s[0] = 0;
  }
  char s[33]; strcpy(s, e.ssid);
  if (printable(s)) strcpy(v.s, s);
  else if (!v.s[0]) {
    if (pr) strcpy(v.s, "any");
    else snprintf(v.s, sizeof(v.s), "hidden %02x%02x", e.mac[4], e.mac[5]);
  }
  v.rssi = (v.rssi * 3 + e.rssi) / 4;
  v.seen = millis();
  return v;
}

static void setChannel(int ch) {
  channel = ch;
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  for (int i = 0; i < NV; i++) voices[i].used = false;
}

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
    if (c == 'q') goToSleep();
    else if (c == '/') setChannel(channel % 13 + 1);
    else if (c == ',') setChannel((channel + 11) % 13 + 1);
    else if (c == 'p') showProbes = !showProbes;
    else if (c == 'm') muted = !muted;
    else if (c == 'h') hopping = !hopping;
    else if (c == '[') divIdx = min(divIdx + 1, 5);
    else if (c == ']') divIdx = max(divIdx - 1, 0);
    else if (c == 'v') pluck = !pluck;
    else if (c == ';') octave = min(octave + 1, 3);
    else if (c == '.') octave = max(octave - 1, -2);
    else if (c == 's') scaleIdx = (scaleIdx + 1) % 4;
    else if (c == 'l') lenIdx = (lenIdx + 1) % 4;
    else if (c == '=') { volume = min(volume + 20, 255); M5Cardputer.Speaker.setVolume(volume); }
    else if (c == '-') { volume = max(volume - 20, 0);   M5Cardputer.Speaker.setVolume(volume); }
  }
}

// ---------- screen: six voice rows, then three control lines ----------
static const int ROWS = 6, ROW_H = 17, BAR_X = 150, BAR_W = 86;

static void draw() {
  canvas.fillSprite(TFT_BLACK);
  uint32_t now = millis();

  // the six strongest voices alive in the last 5 s ...
  int pick[ROWS], np = 0;
  bool taken[NV] = {false};
  for (int r = 0; r < ROWS; r++) {
    int best = -1;
    for (int i = 0; i < NV; i++) {
      if (!voices[i].used || taken[i] || now - voices[i].seen > 5000) continue;
      if (voices[i].probe && !showProbes) continue;
      if (best < 0 || voices[i].rssi > voices[best].rssi) best = i;
    }
    if (best < 0) break;
    taken[best] = true; pick[np++] = best;
  }
  // ... ordered by pitch, highest at top
  for (int a = 0; a < np; a++)
    for (int b = a + 1; b < np; b++)
      if (voicePitch(voices[pick[b]]) > voicePitch(voices[pick[a]])) { int t = pick[a]; pick[a] = pick[b]; pick[b] = t; }

  canvas.setFont(&fonts::FreeMono9pt7b);
  for (int r = 0; r < np; r++) {
    Voice& v = voices[pick[r]];
    int y = 2 + r * ROW_H;
    bool lit = (now - v.flash < 140);
    uint16_t fg = lit ? TFT_BLACK : TFT_WHITE;
    if (lit) canvas.fillRect(0, y - 1, 240, ROW_H - 1, TFT_WHITE);
    canvas.setTextColor(fg);
    char label[14];
    snprintf(label, sizeof(label), "%s%.12s", v.probe ? "?" : "", v.s);
    canvas.drawString(label, 4, y);
    int w = map(constrain(v.rssi, -95, -30), -95, -30, 2, BAR_W);
    canvas.fillRect(BAR_X, y + 5, w, 4, fg);
  }

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(TFT_WHITE);
  char l1[48], l2[48], l3[48];
  snprintf(l1, sizeof(l1), ",/ ch%-2d   h %s   ;. oct%+d", channel, hopping ? "HOP" : "hop", octave);
  snprintf(l2, sizeof(l2), "s %-5s  l x%g  [] /%d  -= vol%d", SCALE_NAMES[scaleIdx], LENS[lenIdx], DIVS[divIdx], volume);
  snprintf(l3, sizeof(l3), "v %s  p %s  m %s  q sleep", pluck ? "pluck" : "click",
           showProbes ? "PROBES" : "probes", muted ? "MUTED" : "mute");
  canvas.drawString(l1, 4, 104);
  canvas.drawString(l2, 4, 114);
  canvas.drawString(l3, 4, 124);
  canvas.pushSprite(0, 0);
}

void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Speaker.setVolume(volume);
  canvas.createSprite(240, 135);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT };
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&onPkt);
  esp_wifi_set_promiscuous(true);
  setChannel(channel);
}

void loop() {
  M5Cardputer.update();
  handleKeys();

  Ev e;
  int budget = 24;
  while (budget-- && pop(e)) {
    if (e.kind == 1 && !showProbes) continue;
    Voice& v = hear(e);
    if (e.kind == 0 && (++v.n % DIVS[divIdx])) continue;   // thin each AP's pulse
    v.flash = millis();
    if (muted) continue;
    int r = constrain((int)e.rssi, -90, -30);
    float f = voicePitch(v);
    if (e.kind == 0) {
      if (pluck) playPluck(f, map(r, -90, -30, 1500, 12000), (int)(4000 * LENS[lenIdx]));
      else M5Cardputer.Speaker.tone(f, (int)(map(r, -90, -30, 4, 40) * LENS[lenIdx]), -1, false);
    } else {
      if (pluck) playPluck(f, 9000, (int)(2000 * LENS[lenIdx]));
      else M5Cardputer.Speaker.tone(f, (int)(30 * LENS[lenIdx]), -1, false);
    }
  }

  static uint32_t lastHop = 0;
  if (hopping && millis() - lastHop > 1500) { setChannel(channel % 13 + 1); lastHop = millis(); }

  static uint32_t lastDraw = 0;
  if (millis() - lastDraw > 60) { draw(); lastDraw = millis(); }
}
