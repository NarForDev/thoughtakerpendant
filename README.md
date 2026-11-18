# Thoughtaker pendant

Firmware for a wearable that records your conversations all day and hands them to your phone, where they are
transcribed and understood without leaving your devices. This repo is the pendant; the phone side is
[thoughtaker-app](https://github.com/NarForDev/thoughtaker-app).

- Records 16 kHz speech to a microSD card as crash-safe 60 s chunks, about 29 MB per hour (roughly 45 days on 32 GB)
- Keeps recording when the phone is away; the phone catches up later over Bluetooth LE
- Pairs once with one phone over an encrypted, bonded link, then syncs in the background
- Sends bookmarked chunks first, then speech, then quiet ones, and deletes a chunk only after the phone has verified its CRC-32
- Mute from the app
- Wake word (in progress): **"Perthro"** starts and stops recording with a 30 to 60 s pre-roll, **"Perthro mark"** bookmarks
  the moment, and **"Perthro forget"** said twice deletes the last 5 minutes. The firmware supports it; the models are still being trained

## Hardware

| Part | Notes |
|---|---|
| [Seeed Studio XIAO ESP32-C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/) | RISC-V 160 MHz, 512 KB SRAM, BLE 5 |
| INMP441 I2S MEMS microphone | 16 kHz mono |
| microSD card module (SPI) + FAT32 card | 32 GB is plenty |
| USB-C power | LiPo battery and enclosure are future work |

| Module pin | XIAO pin |
|---|---|
| INMP441 SCK / WS / SD | D3 / D4 / D5 |
| INMP441 L/R | GND (left channel) |
| microSD CS | D2 (SPI at 4 MHz) |
| microSD SCK / MOSI / MISO | default SPI pins |
| VCC / GND | 3V3 / GND |

## Build and flash

Arduino core for ESP32 **3.3.5**, board `esp32:esp32:XIAO_ESP32C6`.

```sh
arduino-cli core install esp32:esp32@3.3.5
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6 firmware/pendant
arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32C6 -p COM5 firmware/pendant
```

The serial monitor (115200 baud) prints a status line every 10 s; typing `b` adds a bookmark.

**Pairing:** a new pendant accepts a phone for 2 minutes after its first power-up. To pair a different phone, hold the
BOOT button for 3 s; the LED stays on while the pendant is ready to pair, and the old phone is forgotten.

**Wake word:** the firmware builds with it only when `firmware/pendant/wake_models.h` exists. Without that file, the
pendant records whenever it is not muted, as before. See [training/wakeword](training/wakeword/README.md) to train the
models and generate the file.

## How it works

Three FreeRTOS tasks share the work, plus a fourth for the wake word:

1. **mic**: reads I2S samples into a 2 s RAM ring, so a slow SD write never drops audio
2. **writer**: removes the microphone's DC offset, flags speech with an energy VAD, encodes IMA ADPCM and writes whole
   256-byte blocks. Each chunk is written as `P<seq>.PRT` and renamed to `B`/`S`/`Q<seq>.WAV` (bookmarked, speech,
   quiet) once complete, which is the commit point. A chunk interrupted by a power cut is repaired at the next boot
3. **ble**: a NimBLE GATT server that answers the phone's requests (status, set time, list, pull, ack, mute, wake word)
4. **wake**: microWakeWord streaming models on 10 ms spectrogram features

Every chunk is a standard WAV file with a 512-byte header that also carries the sequence number, the time, up to 16
bookmarks and a CRC-32, so the card can be read on any computer.

[docs/protocol.md](docs/protocol.md) is the contract between pendant and app: file format, crash recovery, BLE
service, pairing, every request and event, and the sync procedure. An identical copy lives in the app repo.

## Repo layout

```
docs/protocol.md            pendant <-> phone contract
firmware/pendant/           the firmware (chunk.h is the file format, with a host test in test/)
firmware/kws_bench/         wake-word benchmark on the C6, with the published okay_nabu and vad models
firmware/blink, sd_test, mic_test   bring-up sketches
training/wakeword/          microWakeWord configs and scripts for the three Perthro models
```

Host test for the chunk format and ADPCM encoder:

```sh
g++ -std=c++17 -I firmware/pendant -o chunk_test firmware/pendant/test/chunk_test.cpp && ./chunk_test
```

## Status and limits

- Tested on a breadboard prototype with a Galaxy S24.
- The Perthro wake word models are not trained yet.
- Battery level is not measured; the pendant reports it as unknown.
- Audio on the microSD card is not encrypted until it syncs, so anyone holding the pendant can read unsynced chunks.
- Pairing is Just Works: it is protected by the short pairing window, not by a code, so pair where nobody nearby is
  pairing at the same time.

## Credits

The wake-word front end in `src/microfrontend` comes from TensorFlow Lite Micro, and the benchmark models in
`firmware/kws_bench/models.h` come from [esphome/micro-wake-word-models](https://github.com/esphome/micro-wake-word-models).
Both are Apache-2.0. The wake-word training uses [microWakeWord](https://github.com/kahrendt/microWakeWord).
