# BEACONS

Listening to the WiFi air as sound. An M5Stack Cardputer (original or ADV) piece that puts the radio into promiscuous mode and turns the management traffic around it into music.

Every access point announces itself about ten times a second, each on its own slightly drifting clock. BEACONS gives every access point a fixed pitch and lets those clocks phase against each other. Phones searching for networks they remember sound in a higher register, and the names they call out drift across the screen in bold.

Nothing is stored, logged, or transmitted. It only listens.

## Install

The newest build is always here: [beacons.bin](https://github.com/Oppositecity/beacons/releases/latest/download/beacons.bin)

With [M5Launcher](https://github.com/bmorcelli/Launcher): open WebUI on the device, drag `beacons.bin` into Files, press the rocket to install.

## Keys

| key | does |
|---|---|
| `,` `/` | channel down / up |
| `h` | hop through all channels |
| `;` `.` | octave up / down |
| `s` | scale: pentatonic, harmonic series, whole-tone, chromatic |
| `l` | note length |
| `[` `]` | thin / thicken: each access point sounds every Nth beacon |
| `-` `=` | volume |
| `v` | voice: pluck / click |
| `p` | probe requests on / off |
| `m` | mute |
| `i` | show keys |
| `q` | sleep (reset to wake) |

## Build

Every push to `main` compiles `beacons/beacons.ino` with arduino-cli (ESP32 core 2.0.9, M5Cardputer 1.1.1) and publishes a release.
