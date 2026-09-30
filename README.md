# BEACONS

The air around you as a small orchestra. An M5Stack Cardputer (original or ADV) piece that listens to WiFi and Bluetooth and turns what it hears into warm, struck, natural sound.

Every access point announces itself about ten times a second, each on its own slightly drifting clock. BEACONS gives every device a fixed pitch and lets those clocks phase against each other, while the busier layers of the air arrive as rain, surf and birdsong.

Nothing is stored, logged, or transmitted. It only listens.

## Install

The newest build is always here: [beacons.bin](https://github.com/Oppositecity/beacons/releases/latest/download/beacons.bin)

With [M5Launcher](https://github.com/bmorcelli/Launcher): open WebUI on the device, drag `beacons.bin` into Files, press the rocket to install.

## Layers

| key | layer | what it hears | how it sounds |
|---|---|---|---|
| `1` | beacons | WiFi access points announcing themselves | marimba |
| `2` | probes | phones calling for networks they remember | kalimba |
| `3` | traffic | WiFi data volume on the current channel | rain |
| `4` | floor | the radio noise floor | low drone, like surf |
| `5` | deauth | disconnect frames (rare; often an attack) | birdsong |
| `6` | bluetooth | BLE devices advertising nearby | tongue drum |
| `7` | motion | shaking the Cardputer (ADV motion sensor) | rainstick |

All synthesis is modal (struck-bar physics with natural decays). Pitches are kept between 130 and 2600 Hz, the output runs through a soft limiter, and volume is capped.

## Keys

Both screens: `1`-`7` layer on/off, `tab` switch between play and mixer, `m` mute, `q` sleep (reset to wake).

Play screen:

| key | does |
|---|---|
| `,` `/` | WiFi channel down / up |
| `h` | hop through all channels |
| `;` `.` | octave up / down |
| `s` | scale: pentatonic, harmonic series, whole-tone, chromatic |
| `l` | note length |
| `[` `]` | thin / thicken how often each device sounds |
| `-` `=` | volume |

Mixer screen: `;` `.` select a layer, `-` `=` set its level.

## Build

Every push to `main` compiles `beacons/beacons.ino` with arduino-cli (ESP32 core 2.0.9, M5Cardputer 1.1.1) and publishes a release.
