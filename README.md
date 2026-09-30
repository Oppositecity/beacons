# BEACONS

The air around you as a small band. An M5Stack Cardputer (original or ADV) piece that listens to WiFi and Bluetooth and turns what it hears into warm, struck, harmonic music.

Everything plays in one key, on one tempo grid, over a slowly turning chord progression. Every device gets a fixed role in the chord, and every access point keeps its own slightly drifting clock, so the clocks phase against each other as rhythm.

Nothing is stored, logged, or transmitted. It only listens.

## Install

The newest build is always here: [beacons.bin](https://github.com/Oppositecity/beacons/releases/latest/download/beacons.bin)

With [M5Launcher](https://github.com/bmorcelli/Launcher): open WebUI on the device, drag `beacons.bin` into Files, press the rocket to install.

## Layers

| key | layer | what it hears | its part in the band |
|---|---|---|---|
| `1` | beacons | WiFi access points announcing themselves | marimba, middle voices |
| `2` | probes | phones calling for networks they remember | kalimba, high voices |
| `3` | traffic | WiFi data volume on the current channel | vibraphone arpeggio; busier air, busier arpeggio |
| `4` | floor | the radio noise floor | drone on the chord root, gliding with each change |
| `5` | deauth | disconnect frames (rare; often an attack) | a struck bell chord |
| `6` | bluetooth | BLE devices advertising nearby | tongue drum bass, roots and fifths |
| `7` | motion | shaking the Cardputer (ADV motion sensor) | a strummed chord |

All synthesis is modal (struck-bar physics with natural decays). Pitches stay between 130 and 2000 Hz, the output runs through a soft limiter, and volume is capped.

## Keys

Both screens: `1`-`7` layer on/off, `tab` switch between play and mixer, `m` mute, `q` sleep (reset to wake).

Play screen:

| key | does |
|---|---|
| `,` `/` | WiFi channel down / up |
| `h` | hop through all channels |
| `;` `.` | octave up / down |
| `t` | tempo: 60, 72, 84, 96, 108 |
| `s` | mood: major, dorian, lydian, minor progressions |
| `l` | note length |
| `[` `]` | thin / thicken how often each device plays |
| `-` `=` | volume |

Mixer screen: `;` `.` select a layer, `-` `=` set its level.

## Build

Every push to `main` compiles `beacons/beacons.ino` with arduino-cli (ESP32 core 2.0.9, M5Cardputer 1.1.1) and publishes a release.
