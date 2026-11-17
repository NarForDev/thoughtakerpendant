# Thoughtaker pendant <-> phone protocol, v1

Single source of truth for the firmware (`thoughtakerpendant`) and the app (`thoughtaker-app`).
An identical copy lives in both repos; change both in the same session.
All integers are little-endian. `u8/u16/u32/u64` unsigned, `i64` signed. Offsets are in bytes from 0.

## 1. Audio chunks on the SD card

### 1.1 Audio encoding

| Field | Value |
|---|---|
| Codec | IMA ADPCM, Microsoft variant (WAV format tag `0x0011`) |
| Rate / channels / bits | 16000 Hz, mono, 4 bits per sample |
| Block size (`nBlockAlign`) | 256 bytes |
| Samples per block | 505 (1 in the header + 252 bytes x 2 nibbles) |
| Block layout | `i16` predictor (= sample 0), `u8` step index 0..88, `u8` 0, then 252 bytes; low nibble = earlier sample |
| Data rate | 16000 / 505 x 256 = 8110.9 B/s |

Every block carries its own predictor and step index, so every block decodes on its own. Crash recovery
relies on this: a chunk truncated to whole blocks is a valid file.

The firmware removes DC (the INMP441 offset drifts after power-up) before VAD and before encoding.

### 1.2 Chunk length and boundaries

- A full chunk is **1901 blocks** = 960,005 samples = 60.0003 s; data = 486,656 B; file = 487,168 B.
  The next chunk continues from the next sample, with no gap.
- A chunk ends early, with at least 1 block, when the user mutes, the storage is full, the device shuts down, or the power fails (see 1.6).
  A chunk always holds whole blocks.
- While muted the mic is stopped and nothing is recorded.

### 1.3 Files

| Item | Rule |
|---|---|
| Directory | `/TK/` (FAT32, 8.3 names only) |
| Recording | `/TK/P<seq>.PRT`, where `<seq>` is 7 uppercase hex digits, e.g. `P000001A.PRT` |
| Finalized | `/TK/<class><seq>.WAV`, where `<class>` is `B` (has a bookmark), `S` (has speech, no bookmark), or `Q` (quiet), e.g. `S000001A.WAV` |
| `seq` | u32 counter, unique per `store_id`, starts at 1, +1 per chunk, never reused; max `0x0FFFFFFF` (7 hex digits, about 510 years) |

The class prefix lets the device sort the sync order from directory names alone, without opening files.
At most one `.PRT` file exists while the device runs, plus one left behind by each SD write error (see 1.6 rule 6).

**Not encrypted:** chunks are stored as plain WAV files. Anyone holding the pendant can remove the card and play
every chunk that has not been synced and acked. Encryption at rest is deferred.

`seq` and `boot_id` are kept in NVS. At boot, `next_seq = max(NVS next_seq, highest seq on card + 1)`.
NVS `next_seq` is written when each chunk opens. `boot_id` is a u32 that goes up by 1 on every boot.
`store_id` is a random u32 created the first time NVS is empty, which means it changes after a full flash erase.
The phone identifies a chunk by **(`store_id`, `seq`)**, taking `store_id` from the chunk's own header (offset 64).
After a full flash erase, chunks still on the card carry the old `store_id`, which differs from the one in GET_STATUS.
The device lists, pulls and ACKs them by `seq` alone like any other chunk; the phone checks only `seq` against the LIST entry.

### 1.4 File layout: a fixed 512-byte header, then audio

The header is always exactly 512 bytes, so the audio starts on an SD sector boundary.

| Off | Type | Field | Value |
|---|---|---|---|
| 0 | 4c | `RIFF` | |
| 4 | u32 | riff size | `504 + data_size` |
| 8 | 4c | `WAVE` | |
| 12 | 4c | `fmt ` | |
| 16 | u32 | fmt size | 20 |
| 20 | u16 | format tag | `0x0011` |
| 22 | u16 | channels | 1 |
| 24 | u32 | sample rate | 16000 |
| 28 | u32 | avg bytes/s | 8110 (for information only) |
| 32 | u16 | block align | 256 |
| 34 | u16 | bits per sample | 4 |
| 36 | u16 | cbSize | 2 |
| 38 | u16 | samples per block | 505 |
| 40 | 4c | `fact` | |
| 44 | u32 | fact size | 4 |
| 48 | u32 | sample count | `blocks x 505` |
| 52 | 4c | `tkmd` | Thoughtaker metadata chunk |
| 56 | u32 | tkmd size | 444 |
| 60 | u16 | tkmd version | 1 |
| 62 | u16 | flags | bit0 `has_speech`, bit1 `bookmarked`, bit2 `time_synced`, bit3 `recovered`; other bits 0 |
| 64 | u32 | store_id | |
| 68 | u32 | seq | must match the file name |
| 72 | u32 | boot_id | boot in which the chunk was recorded |
| 76 | u64 | start_uptime_ms | ms since boot (`esp_timer_get_time()/1000`) at the chunk's first sample |
| 84 | i64 | epoch_offset_ms | Unix epoch ms minus uptime ms; valid only if `time_synced` is set, otherwise 0 |
| 92 | u32 | crc32 | see 1.5 |
| 96 | u8 | bookmark_count | 0..16 |
| 97 | 3B | reserved | 0 |
| 100 | u32[16] | bookmark_ms | offset of each bookmark from the chunk start, in ms; unused entries are 0 |
| 164 | 340B | reserved | 0 |
| 504 | 4c | `data` | |
| 508 | u32 | data size | `blocks x 256` |
| 512 | | audio blocks | |

Flag and field rules:

- `has_speech`: the VAD detected speech at any point in the chunk. `bookmarked` means `bookmark_count > 0`.
  After 16 bookmarks, later ones are dropped, and the flag stays set.
- Time: `start_epoch_ms = start_uptime_ms + epoch_offset_ms`. The device writes the offset that is in effect when it last writes the header.
  If time was never set in the chunk's boot, `time_synced = 0` and the phone resolves the time like this:
  1. If the phone set the time during that same `boot_id` (it stores `(store_id, boot_id) -> epoch_offset_ms` for every SET_TIME it sends),
     it applies that offset. Uptime is monotonic within a boot, so this is exact.
  2. Otherwise the time is **unknown**. The phone orders the chunk by `seq` and shows it as "time unknown",
     bounded by the nearest chunks whose time is known.
- The **valid file length** is `512 + data_size`. Bytes past it (left over by recovery) are ignored and never transferred.

### 1.5 CRC32

Algorithm: CRC-32/ISO-HDLC (zlib `crc32`, reflected polynomial `0xEDB88320`, init and xorout `0xFFFFFFFF`, check("123456789") = `0xCBF43926`).

Input, in this order:
1. the audio bytes `file[512 .. 512 + data_size)`, then
2. the final 512-byte header, with bytes 92..95 (the crc field) set to 0.

Audio comes first so the device can compute the CRC incrementally while recording and finish it on the header at finalize, with no read-back.
On the device: `crc = esp_rom_crc32_le(0, ...)`, then `crc = esp_rom_crc32_le(crc, ...)` for each later piece (zlib-compatible).
On the phone: zlib-style `crc32(bytes, previousCrc)` over the same two pieces.

### 1.6 Writing and crash safety

Writer rules for the firmware:
1. Open `P<seq>.PRT` and write the full 512-byte header with the known fields (`riff size`, `fact`, and `data size` are 0, crc is 0). Then `flush()`.
2. Encoded blocks go to a RAM ring of at least 2 s, about 16 KB. The SD write task writes **whole 256-byte blocks only**
   and calls `flush()` at least every 2 s, because FAT only updates the file size on flush or close.
   The first write after mounting stalled for 713 ms on the bench, and this ring absorbs stalls like that.
3. When a flag changes, a bookmark is added, or the time is set, rewrite the 512-byte header in place and `flush()`.
   This happens rarely, and it keeps the metadata through a power loss.
4. Finalize: fill in the sizes, `fact`, and flags; compute the crc; write the header; `close()`; then **rename** to `<class><seq>.WAV`.
   The rename rewrites one 8.3 directory entry, which makes it the commit point.
5. If storage is full (free space under 2 chunks), finalize, stop recording, and set `storage_full`.
   The device never deletes chunks that have not been acked.
6. If an open, write, or rename fails, close the chunk (it stays a `.PRT` until recovery at the next boot), set `sd_error`,
   and try a new chunk every 5 s; `sd_error` clears when one opens. LIST, PULL and ACK keep serving finalized chunks:
   they return `ERR_IO` only when no card was mounted at boot or that operation's own SD access fails.

Recovery, at boot before recording starts, for each `*.PRT`:

| Condition | Action |
|---|---|
| length < 512, or bytes 0..3 are not `RIFF`, or bytes 52..55 are not `tkmd`, or the tkmd `seq` does not match the name | delete |
| `blocks = floor((length - 512) / 256)` is 0 | delete |
| otherwise | set the sizes and `fact` from `blocks`, set `recovered`, drop any `bookmark_ms` at or beyond the duration, compute the crc by reading the file back, write the header, close, and rename to `.WAV` |

Recovery is idempotent: a power loss during recovery leaves the `.PRT` in place, and recovery runs again on the next boot.
Worst-case audio loss on a power cut is the unflushed RAM ring plus one flush interval, about 4 s.
A power cut can still corrupt the FAT itself. That is out of scope.

## 2. BLE GATT

### 2.1 Service

| Item | UUID | Properties | Security |
|---|---|---|---|
| Service | `3f66b6a2-8fd5-423c-9ef1-494e6f3b9100` | primary, included in advertising | |
| RX (phone -> device) | `3f66b6a2-8fd5-423c-9ef1-494e6f3b9101` | Write (with response) only | encrypted (`PROPERTY_WRITE_ENC`) |
| TX (device -> phone) | `3f66b6a2-8fd5-423c-9ef1-494e6f3b9102` | Notify (CCCD) | encrypted (`PROPERTY_READ_ENC`; the core's NimBLE `notify()` skips unencrypted links) |

- Advertising: connectable, with the service UUID and the name `Thoughtaker`. The device allows one connection and stops advertising while connected.
- Stack: NimBLE (esp32 core 3.3.5 on the C6). L2CAP CoC is compiled out, so all data goes over GATT notifications.

### 2.2 Link parameters

| Parameter | Requirement |
|---|---|
| ATT MTU | Device preferred 517 (`BLEDevice::setMTU(517)`); phone calls `requestMtu(517)`. **Minimum 247**: below that, every request returns `ERR_MTU`. |
| DLE | 251-byte LL payload; Android negotiates it automatically when both sides support it |
| PHY | Phone calls `setPreferredPhy(2M, 2M)`. 1M is allowed and is only slower. |
| Connection priority | Phone uses `CONNECTION_PRIORITY_HIGH` while pulling and `BALANCED` when idle |

### 2.3 Pairing and bonding

- LE Secure Connections only (no legacy pairing), bonding on, IO capability NoInputNoOutput, so the method is **Just Works**.
  Firmware: `BLESecurity::setCapability(ESP_IO_CAP_NONE)`, `setAuthenticationMode(true /*bond*/, false /*mitm*/, true /*sc*/)`.
- The phone calls `createBond()` before its first GATT operation. Any RX write on an unencrypted link fails with ATT error `0x05` (Insufficient Authentication) when the phone has no bond with the device, or `0x0F` (Insufficient Encryption) when it has one (NimBLE's rule).
- **Pairing window:** the device accepts a *new* bond only while its pairing window is open, which takes action on the pendant:
  - for 120 s after power-on, but only until a phone has bonded for the first time (an NVS flag, so a reboot never reopens it);
  - for 120 s after the user holds the BOOT button for 3 s. This also deletes every bond and drops the current link,
    so the old phone must re-pair (on Android, forget the pendant first).
  The status LED stays on while the window is open. The window closes as soon as a phone bonds, so one bond is kept.
- Outside the window: a link that encrypts without an existing bond is disconnected, and its RX writes are ignored until then.
  A peer that tries to pair again with a bonded address is refused (the bond is kept).
- The phone shows every device advertising the service and pairs only the one the user picks.
  It saves that device only if GET_STATUS then reports `pairing_window_open`; otherwise it tells the user to hold the pendant's BOOT button for 3 s.
- **MITM limitation:** Just Works does not authenticate the peer. An attacker who is in radio range *during pairing* can sit in the middle or pair first.
  Once a bond exists, the link is AES-CCM encrypted and integrity-protected. The pairing window keeps the exposure short.
  Real MITM protection needs an out-of-band secret (for example a QR code on the device, used for LE OOB) and is deferred.

## 3. Operations

### 3.1 Framing

Every RX write is one request. Every TX notification is one frame of at most `min(MTU - 3, 512)` bytes: ATT caps an attribute value at 512 bytes even when the MTU allows more.

RX request: `[op u8][req_id u8][payload]`. `req_id` is chosen by the phone and echoed in the response.

TX frames:

| Kind (byte 0) | Layout |
|---|---|
| `0x01` RESP | `[0x01][op u8][req_id u8][status u8][payload]` |
| `0x02` DATA | `[0x02][offset u32][bytes]`: file bytes for the active PULL; at most `min(MTU - 3, 512) - 5` bytes (507 at MTU 517) |
| `0x03` EVENT | `[0x03][event u8][payload]` |

Sequencing: **one request at a time.** After its write is acknowledged, the phone waits for the RESP with the matching `req_id`
before sending the next request. EVENT frames can arrive at any time, including between DATA frames.
Every request gets exactly one RESP. A payload follows only when `status = 0`.

### 3.2 Status codes

| Code | Name | Meaning |
|---|---|---|
| 0x00 | OK | |
| 0x01 | ERR_UNKNOWN_OP | the op is not defined |
| 0x02 | ERR_BAD_LENGTH | the payload length is wrong for the op (a write shorter than 2 bytes gets RESP op=0, req_id=0) |
| 0x03 | ERR_BAD_VALUE | a field is out of range |
| 0x04 | ERR_NOT_FOUND | no finalized chunk with that seq |
| 0x05 | ERR_RANGE | PULL offset or length is outside the valid file length, or length > 65536 |
| 0x06 | ERR_CRC | the ACK crc does not match the stored crc; the chunk is kept |
| 0x07 | ERR_IO | SD card missing or a read, write, or delete failed |
| 0x08 | ERR_UNSUPPORTED | the feature is not built into this firmware (for example, wake word) |
| 0x09 | ERR_MTU | the negotiated ATT MTU is below 247 |

### 3.3 Requests

| Op | Name | Request payload | RESP payload (status OK) | Notes |
|---|---|---|---|---|
| 0x01 | GET_STATUS | none | Status, 32 B (below) | |
| 0x02 | SET_TIME | `epoch_ms u64` | none | Sets `epoch_offset = epoch_ms - uptime_ms` for this boot; not kept across reboots |
| 0x03 | LIST | `max_entries u8` (>=1) | `total_pending u32`, `count u8`, then `count` x Entry (9 B) | Order: class B, then S, then Q; within a class, ascending seq. `count = min(max_entries, (min(MTU-3, 512)-9)/9, total_pending)` (55 at MTU 517). Excludes the chunk being recorded. |
| 0x04 | PULL | `seq u32`, `offset u32`, `length u32` | none | DATA frames cover `[offset, offset+length)` in order, and **the RESP comes after the last DATA frame**. 1 <= length <= 65536, and offset+length <= the valid file length. |
| 0x05 | ACK | `seq u32`, `crc32 u32` | none | The device deletes the chunk only if `crc32` equals the stored header crc. ACK of a seq that does not exist returns **OK** (idempotent: a retry after a lost RESP must not fail). |
| 0x06 | SET_MUTE | `muted u8` (0/1) | none | Muting finalizes the current chunk and stops the mic. Kept in NVS across reboots. |
| 0x07 | SET_WAKE_WORD | `enabled u8` (0/1) | none | Kept in NVS. ERR_UNSUPPORTED if no wake-word engine is built in. |

Status payload (32 B):

| Off | Type | Field |
|---|---|---|
| 0 | u16 | proto_version = 1 (the phone refuses other values) |
| 2 | u8 | state: bit0 `recording`, bit1 `muted`, bit2 `wake_word_enabled`, bit3 `time_synced` (this boot), bit4 `storage_full`, bit5 `sd_error`, bit6 `pairing_window_open` |
| 3 | u8 | battery_pct; `0xFF` = unknown (placeholder until the D0 ADC is wired) |
| 4 | u32 | store_id |
| 8 | u32 | boot_id |
| 12 | u64 | uptime_ms |
| 20 | u32 | free_kib |
| 24 | u32 | total_kib |
| 28 | u32 | pending_chunks (finalized `.WAV` count) |

`recording` = not muted, and not `storage_full`, and not `sd_error`.
`sd_error` = no usable card at boot, or the last chunk open or write failed (the device retries every 5 s, see 1.6 rule 6).
This firmware has no wake-word engine: SET_WAKE_WORD always returns ERR_UNSUPPORTED and `wake_word_enabled` is 0.

LIST Entry (9 B): `seq u32`, `size u32` (valid file length), `flags u8` (bit0 `has_speech`, bit1 `bookmarked`, taken from the class prefix).

### 3.4 Events

| Event | Name | Payload | When |
|---|---|---|---|
| 0x01 | CHUNK_READY | `seq u32`, `size u32`, `flags u8` (as in a LIST entry) | after the rename to `.WAV` |
| 0x02 | BOOKMARK | `seq u32` (chunk being recorded), `offset_ms u32` (from the chunk start), `uptime_ms u64` | wake word detected |

### 3.5 Sync procedure (phone)

1. Connect, make sure the link is bonded and encrypted, request MTU 517 and 2M PHY, and subscribe to TX.
2. `SET_TIME`, then save `(store_id, boot_id) -> epoch_offset`, then `GET_STATUS`.
3. Loop: `LIST` -> for each entry, `PULL` pieces of up to 65536 B into a temp file -> check that the crc32 matches header bytes 92..95 -> commit to app storage -> `ACK`.
   Do LIST again after a batch, or after CHUNK_READY.
4. Commit the chunk durably **before** sending the ACK. If the crc does not match, delete the temp file and pull again from offset 0.

### 3.6 Timeouts and resumption

- The phone times out a RESP after 2 s for SET_TIME, SET_MUTE, SET_WAKE_WORD, GET_STATUS, and ACK; after 10 s for LIST (the device scans the directory);
  and for PULL after 5 s with no DATA frame or RESP. On timeout the phone disconnects and reconnects, which clears all device state.
- The device keeps **no transfer state** across a disconnect: it stops streaming and closes the file.
  Each DATA frame carries its absolute offset. The phone keeps `(store_id, seq, contiguous_bytes_received)` and the temp file,
  and after reconnecting it sends `PULL seq offset=contiguous_bytes_received`.
  If a DATA offset is not the next expected one, the phone treats the window as failed and pulls again from its contiguous point.
- A chunk is deleted only by an ACK with a matching crc, so a pull can always be resumed or restarted.
  `ERR_NOT_FOUND` on PULL means the chunk was already acked (for example, an ACK whose RESP was lost): discard the partial file.
- Device notify backpressure: if `notify()` fails (reported synchronously, in the calling task, through `BLECharacteristicCallbacks::onStatus` with `ERROR_GATT`, for example when the stack is out of buffers),
  the device waits a few ms and resends **the same** frame. It never skips a DATA frame.

## 4. Throughput

| Quantity | Value |
|---|---|
| ADPCM file rate | 487,168 B / 60.0003 s = 8.12 kB/s |
| Per hour | 29.2 MB |
| Per 16-h waking day | 468 MB |
| Per 24 h | 702 MB; a 32 GB card holds about 45 days (the FAT32 limit of 65,536 entries per directory also lands near 45 days) |
| DATA payload per notification | 507 B at MTU 517, so 961 notifications per full chunk |
| 2M PHY ceiling | Each encrypted 251-B LL PDU takes 266 B on air = 1064 us, plus 2 x 150 us IFS and a 44 us empty ack, about 1.41 ms per PDU, so at most about 178 kB/s |
| Realistic ESP32-C6 to Android | 50-100 kB/s with 2M PHY, DLE, and MTU 517 (the phone's connection-event length is the limit); plan for **60 kB/s**; worst case about 25 kB/s (1M PHY, no DLE) |

| At | Speed vs real time | 1 chunk | 16-h day of audio |
|---|---|---|---|
| 100 kB/s | 12x | 4.9 s | 1.3 h |
| 60 kB/s | 7.4x | 8.1 s | 2.2 h |
| 25 kB/s | 3.1x | 19.5 s | 5.2 h |

What this means:
- With the phone nearby, sync keeps up easily: the radio is busy about 14% of the time at 60 kB/s.
- After a day away from the phone, catching up takes hours, so the sync order (bookmarked, then speech, then quiet) is what gets
  the useful audio to the phone first. Quiet chunks are most of the bytes and are the first thing to cut later
  (for example, the device dropping chunks with no speech). That is not part of v1.
- Even the worst case (24 h at 25 kB/s, about 7.8 h) fits in an overnight charge with the phone nearby.
- The phone receives about 470 MB/day of ADPCM. It should transcribe and then delete or transcode the audio, not keep the raw files.
- The SD write load (8 kB/s) is trivial. Write stalls, not bandwidth, are why the RAM ring exists.
