// BEACONS v0.4 — listening sketch for M5Stack Cardputer (original + ADV)
// Every access point on the current channel becomes a voice.
// Beacons (~10/sec per AP) sound at a pitch hashed from the AP's MAC,
// thinned so each AP pulses on its own real clock; closer = louder.
// Probe requests (phones calling for remembered networks) sound high,
// and their names appear in bold.
//
// keys:  , /  channel down / up        h  auto-hop channels
//        ; .  octave up / down         s  cycle scale
//        l    cycle note length        - =  volume down / up
//        p    probes on/off            m  mute
//        [ ]  thin / thicken: each AP sounds every Nth beacon (default 10 = ~1/sec)
//        v    voice: pluck (sine, decaying) / click (square)
//        i    show keys (two pages)    q  sleep: sound, radio, screen off (reset to wake)
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

static int  channel    = 6;
static bool showProbes = true;
static bool muted      = false;
static bool hopping    = false;
static int  octave     = 0;                    // shifts every voice
static int  scaleIdx   = 0;                    // pentatonic, harmonic, whole-tone, chromatic
static int  lenIdx     = 1;                    // note length multiplier
static int  volume     = 140;
static const float LENS[] = {0.5f, 1.0f, 2.0f, 4.0f};
static const char* SCALE_NAMES[] = {"penta", "harm", "whole", "chrom"};
static const int DIVS[] = {1, 2, 5, 10, 20, 50};
static int  divIdx     = 3;                    // every 10th beacon per AP
static bool pluck      = true;
static int  helpPage   = 1;                    // keys shown at boot; 0 = hidden

// per-AP beacon counters, so each AP keeps its real timing but slower
struct Ctr { uint8_t mac[6]; uint32_t n; bool used; };
static const int CN = 48;
static Ctr ctrs[CN];
static uint32_t tick(const uint8_t* mac) {
  int freeI = -1;
  for (int i = 0; i < CN; i++) {
    if (ctrs[i].used && memcmp(ctrs[i].mac, mac, 6) == 0) return ++ctrs[i].n;
    if (!ctrs[i].used && freeI < 0) freeI = i;
  }
  if (freeI < 0) freeI = esp_random() % CN;
  memcpy(ctrs[freeI].mac, mac, 6); ctrs[freeI].n = esp_random() % 50; ctrs[freeI].used = true;
  return ctrs[freeI].n;
}

// pluck voices: sine with exponential decay, rendered into a small pool
static const int VOICES = 6, VLEN = 8000, VRATE = 16000;
static int16_t vbuf[VOICES][VLEN];
static int nextVoice = 0;
static void playPluck(float freq, int amp, int len) {
  int v = nextVoice; nextVoice = (nextVoice + 1) % VOICES;
  M5Cardputer.Speaker.stop(v);
  if (len > VLEN) len = VLEN;
  float ph = 0, dph = 2.0f * PI * freq / VRATE, env = 1.0f, k = expf(-5.0f / len);
  for (int i = 0; i < len; i++) {
    float a = (i < 40) ? env * i / 40.0f : env;   // tiny attack, no click
    vbuf[v][i] = (int16_t)(amp * a * sinf(ph));
    ph += dph; if (ph > 2 * PI) ph -= 2 * PI;
    env *= k;
  }
  M5Cardputer.Speaker.playRaw(vbuf[v], len, VRATE, false, 1, v, true);
}

// ---------- names currently alive on this channel ----------
struct Name { char s[33]; uint8_t mac[6]; bool probe; uint32_t t; bool used; };
static const int NN = 10;
static Name names[NN];

M5Canvas canvas(&M5Cardputer.Display);

static uint32_t hashMac(const uint8_t* m) {
  uint32_t h = 2166136261u;
  for (int i = 0; i < 6; i++) { h ^= m[i]; h *= 16777619u; }
  return h;
}

// each device gets a fixed place in the current scale
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

// ---------- promiscuous callback (runs in wifi task: keep it tiny) ----------
static void onPkt(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  auto* p = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* f = p->payload;
  int len = p->rx_ctrl.sig_len;
  if (len < 24) return;

  uint8_t sub = (f[0] >> 4) & 0x0F;
  uint8_t kind; int tag;
  if (sub == 8)      { kind = 0; tag = 36; }  // beacon: tags after 12 fixed bytes
  else if (sub == 4) { kind = 1; tag = 24; }  // probe request
  else return;

  Ev e;
  e.kind = kind;
  e.rssi = p->rx_ctrl.rssi;
  memcpy(e.mac, f + 10, 6);                   // transmitter address
  e.ssid[0] = 0;
  if (len > tag + 2 && f[tag] == 0) {          // tag 0 = SSID
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
  return any;                                  // hidden networks: sound, no name
}

static void remember(const Ev& e) {
  char s[33]; strcpy(s, e.ssid);
  if (!printable(s)) return;
  int slot = -1, oldest = 0;
  for (int i = 0; i < NN; i++) {
    if (names[i].used && memcmp(names[i].mac, e.mac, 6) == 0 && names[i].probe == (e.kind == 1)) { slot = i; break; }
    if (!names[i].used) { if (slot < 0) slot = i; }
    else if (names[i].t < names[oldest].t) oldest = i;
  }
  if (slot < 0) slot = oldest;
  strcpy(names[slot].s, s);
  memcpy(names[slot].mac, e.mac, 6);
  names[slot].probe = (e.kind == 1);
  names[slot].t = millis();
  names[slot].used = true;
}

static void setChannel(int ch) {
  channel = ch;
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  for (int i = 0; i < NN; i++) names[i].used = false;
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

static const char* HELP[2][6] = {
  {", /  channel", "; .  octave", "s    scale", "l    note length", "[ ]  thin / thick", "h    hop channels"},
  {"- =  volume", "v    voice", "p    probes", "m    mute", "q    sleep", "i    next / close"}
};

static void handleKeys() {
  if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) return;
  auto ks = M5Cardputer.Keyboard.keysState();
  for (char c : ks.word) {
    if (c == 'i') { helpPage = (helpPage + 1) % 3; continue; }
    helpPage = 0;                              // any other key closes the key list
    if (c == 'q') goToSleep();
    else if (c == '/')      setChannel(channel % 13 + 1);
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

static void draw() {
  canvas.fillSprite(TFT_BLACK);
  canvas.setTextColor(TFT_WHITE);
  if (helpPage) {
    canvas.setFont(&fonts::FreeMono9pt7b);
    for (int i = 0; i < 6; i++) canvas.drawString(HELP[helpPage - 1][i], 8, 6 + i * 20);
    canvas.drawString(helpPage == 1 ? "keys 1/2" : "keys 2/2", 150, 118);
    canvas.pushSprite(0, 0);
    return;
  }
  uint32_t now = millis();
  for (int i = 0; i < NN; i++) {
    if (!names[i].used || now - names[i].t > 4000) continue;
    canvas.setFont(names[i].probe ? &fonts::FreeMonoBold9pt7b : &fonts::FreeMono9pt7b);
    uint32_t h = hashMac(names[i].mac);
    int w = canvas.textWidth(names[i].s);
    int x = (w < 236) ? (int)(h % (236 - w)) : 0;
    int y = 4 + (int)((h >> 12) % 100);
    canvas.drawString(names[i].s, x, y);
  }
  canvas.setFont(&fonts::FreeMono9pt7b);
  char st[64];
  snprintf(st, sizeof(st), "%d%s %s %+d x%g /%d%s%s%s", channel, hopping ? "~" : "",
           SCALE_NAMES[scaleIdx], octave, LENS[lenIdx], DIVS[divIdx], pluck ? "" : " clk",
           muted ? " mute" : "", showProbes ? "" : " -p");
  canvas.drawString(st, 2, 118);
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
  int budget = 24;                             // don't let a dense channel starve the screen
  while (budget-- && pop(e)) {
    if (e.kind == 1 && !showProbes) continue;
    remember(e);
    if (muted) continue;
    int r = constrain((int)e.rssi, -90, -30);
    if (e.kind == 0) {
      if (tick(e.mac) % DIVS[divIdx]) continue;             // thin each AP's pulse
      if (pluck) playPluck(pitchFor(e.mac, 0), map(r, -90, -30, 1500, 12000), (int)(4000 * LENS[lenIdx]));
      else M5Cardputer.Speaker.tone(pitchFor(e.mac, 0), (int)(map(r, -90, -30, 4, 40) * LENS[lenIdx]), -1, false);
    } else {
      if (pluck) playPluck(pitchFor(e.mac, 3), 9000, (int)(2000 * LENS[lenIdx]));
      else M5Cardputer.Speaker.tone(pitchFor(e.mac, 3), (int)(30 * LENS[lenIdx]), -1, false);
    }
  }

  static uint32_t lastHop = 0;                 // hop: one channel every 1.5 s
  if (hopping && millis() - lastHop > 1500) { setChannel(channel % 13 + 1); lastHop = millis(); }

  static uint32_t lastDraw = 0;
  if (millis() - lastDraw > 120) { draw(); lastDraw = millis(); }
}
