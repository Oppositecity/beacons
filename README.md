# BEACONS

Listening to the air. An M5Stack Cardputer (original or ADV) piece that listens to WiFi and Bluetooth and turns what it hears into struck, ringing sound.

Nothing is composed. Every sound is a measurement: notes sound the moment a signal arrives, pitch is how close its source is, and the whole field shifts as you walk through a room.

Nothing is stored, logged, or transmitted. It only listens.

## Install

The newest build is always here: [beacons.bin](https://github.com/Oppositecity/beacons/releases/latest/download/beacons.bin)

With [M5Launcher](https://github.com/bmorcelli/Launcher): open WebUI on the device, drag `beacons.bin` into Files, press the rocket to install.

## What each part of the sound means

| sound | measurement |
|---|---|
| when a note sounds | the moment a signal arrives |
| pitch | how close the source is: each device climbs the harmonic series as you approach it, about one step per 4-5 dB |
| the fundamental under it all | the WiFi channel you're hearing (ch1 = 55 Hz ... ch13 = 110 Hz) |
| a slight sharp or flat bend | the source getting closer or farther, like a doppler shift |
| loudness, brightness, ring length | signal strength |
| timbre | the kind of signal |

Each layer's loudness is fixed, so loudness only ever means signal strength.

## Layers

| key | layer | what it hears | timbre |
|---|---|---|---|
| `1` | aps | WiFi access points announcing themselves, each on its own clock | marimba |
| `2` | probes | phones calling for networks they remember | kalimba, an octave up |
| `3` | data | WiFi data frames; every Nth frame sounds, ringing longer for bigger frames | vibraphone |
| `4` | floor | the radio noise floor, and how many devices are present | drone: loudness = noise floor, one harmonic per device present |
| `5` | deauth | disconnect frames (rare; often an attack) | a struck bell |
| `6` | ble | Bluetooth devices advertising nearby | tongue drum, low register |

Pitches stay between 110 and 2000 Hz, the output runs through a soft limiter, and volume is capped.

## The screen

Every mark is a measurement.

- **Rows:** the nearest sources, up to three WiFi and three Bluetooth, closest (highest pitch) at the top. A row flashes when it sounds, the bar is signal strength, and `+` / `-` means approaching / receding. `?` = probe, `*` = Bluetooth.
- **Layer labels:** each layer's key, name and a live count: access points present, phones probing, data frames per second, noise floor in dBm, disconnects heard, Bluetooth devices present. UPPERCASE = on. A label flashes when its layer sounds.
- **Bottom line:** channel and its frequency in MHz, hop, thinning, volume, mute (`M` when muted).

## Keys

| key | does |
|---|---|
| `1`-`6` | layer on / off |
| `,` `/` | WiFi channel down / up |
| `h` | hop through all channels |
| `[` `]` | thin / thicken how often each source sounds |
| `-` `=` | volume |
| `m` | mute |
| `q` | sleep (reset to wake) |

## Build

Every push to `main` compiles `beacons/beacons.ino` with arduino-cli (ESP32 core 2.0.9, M5Cardputer 1.1.1) and publishes a release.
