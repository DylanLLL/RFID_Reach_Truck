# RFID Reach Truck

Firmware for an RFID-based rack location capture system at WH Marunda. Design source of truth is
`RFID Reach Truck Project.pdf` (kept outside the repo); this file records the architecture decisions
made since, several of which supersede the PDF.

## Problem

During put-away, reach truck drivers scan a QR code label on the racking beam to record the pallet
location in Stockholm/WMS. Beams carry many labels in-line, so drivers frequently scan the wrong one
— by miscounting, or because the Zebra handheld picks up a neighbouring label at distance. The wrong
location enters WMS and surfaces later as "missing" stock.

This system reads the location automatically at fork insertion and displays the correct QR code on an
OLED in the cabin. The driver's SOP is unchanged: they still scan a QR code, just one in front of them.

## Hardware

Two ESP32-S3-DevKitC-1 units per truck (16 MB flash, 8 MB OPI PSRAM), one fleet of 10 trucks.

**Fork unit** — mounted on the fork carriage, battery powered (18650 pack):
- Invelion RF200 UHF RFID reader, UART TTL @ 115200, 902-928 MHz (Indonesia allocation)
- External ceramic antenna, 0-2 m range, mounted **angled** rather than level with the fork — a pallet
  blocks line of sight to the tag at level
- 3/6 dB attenuator, deliberately shortening range so neighbouring bays aren't picked up
- HY-SRF05 ultrasonic sensor, detects the beam and gates the read window

**Cabin unit** — mains/accessory powered, mounted in the cab:
- 3.2" ILI9341 **TFT LCD**, 240x320, no touch — displays the QR code for the driver's Zebra handheld.
  (The PDF/BOM call this a "3.5 inch OLED"; it is neither 3.5" nor an OLED. Corrected here.)
- SPI wired to the ESP32-S3 **native IO_MUX pins for FSPI/SPI2**: CLK=12, MOSI=11, MISO=13, CS=10,
  plus DC=14, RST=21, BL=16. Staying on the IO_MUX set routes SPI directly instead of through the
  GPIO matrix, which is what makes 40 MHz clean. Moving these to arbitrary GPIOs means dropping the
  clock. Note GPIO 33-37 are unusable on this board — they carry the octal PSRAM.

**Tags** — passive anti-metal UHF, one per storage location, unique EPC.

## Architecture decisions

### 1. Local lookup table, not lookup-on-read — SUPERSEDES the PDF

The PDF has the fork unit querying Firebase per read. Rejected: it puts a cloud round trip on the
critical path, on the battery-powered device, inside a metal racking canyon. WiFi roaming on ESP32
costs 1-3 s, TLS handshake another second, and a WiFi dropout fails the system exactly when it is in use.

EPC -> location is a **static** mapping — a beam does not move; the row changes only when a human
re-tags one. So the whole table lives on-device and Firebase becomes a **sync source, not a query source**.

Storage: 12-byte EPC key + 4-byte packed `{rack, bay, level, side}`, formatted to text only at display
time. 16 bytes x 33,700 = **527 KB** — held in PSRAM (too big for the 512 KB internal SRAM),
persisted to a LittleFS partition on the 16 MB flash. Sorted array, binary search, ~16 comparisons.

Firebase sync happens on the cabin unit, in an idle window (start of shift / parked / charging),
gated on a version counter.

### 2. Table lives on the CABIN unit; fork unit is a dumb sensor

The fork unit is bolted to the carriage in a sealed enclosure on battery — the hardest device in the
system to reflash. So it holds as little changeable logic as possible. It also has no WiFi, and no
good way to receive a 527 KB table.

- **Fork unit**: ultrasonic gates a read window -> collect tag inventory -> send
  `{seq, [(epc, rssi, hits)], gate_state}` over ESP-NOW. Under 60 bytes, well inside the 250-byte limit.
- **Cabin unit**: RSSI arbitration, table lookup, QR generation, staleness enforcement, Firebase sync.

Consequence: arbitration policy can be tuned during the pilot by reflashing the accessible unit.

### 3. ESP-NOW for the inter-ESP32 link

Chosen over WiFi MQTT. Consistency matters more than latency here. MQTT adds AP association on both
ends plus broker reachability — three failure points outside the vehicle. ESP-NOW keeps the link
entirely on the truck and works when warehouse WiFi is down, and skipping association is a large
battery win for the fork unit.

**Channel constraint**: ESP-NOW and WiFi STA share one radio and one channel. A device associated to
an AP has its channel pinned to that AP's, which deafens ESP-NOW peers on another channel. This is
why the fork unit never associates, and why the cabin unit's Firebase sync happens in a defined idle
window rather than continuously.

### 4. Two tags per beam are two DIFFERENT locations

33,700 storage locations already counts 2 per level, so each level has 2 tags representing the left
and right pallet positions, roughly 1.2 m apart. RSSI arbitration is therefore load-bearing — it
distinguishes two tags on the same beam, not just adjacent bays. Expect this to need tuning against
real racking, in heavy multipath.

Policy: sample over a 150-300 ms window, require several hits per EPC, and demand a **minimum RSSI
margin** between the top two candidates. If the margin is thin, display nothing.

### 5. EPC encoding — deferred, not rejected

Encoding rack/bay/level/side directly into the 96-bit EPC would remove the lookup entirely. Deferred
until the RF200's EPC write capability is tested on real tags. If adopted, keep a version nibble in
the EPC for future migration, and keep Firebase authoritative for exceptions (re-tagged beams,
renamed locations).

Decide before 33,700 tags are encoded.

## R200 driver (`lib/R200/`)

Adapted from Alastair Aitchison's Playful Technology demo. PlatformIO only sees a `lib/` library if
it is in its own subdirectory, hence `lib/R200/R200.{h,cpp}` — files loose in `lib/` are silently
ignored and `#include "R200.h"` will not resolve.

Single-poll response frame, with the offsets that matter:

    AA 02 22 00 11 C7 30 00 E2 80 ... A7 11 9B 29 DD
    [0] header  [1] type  [2] cmd  [3][4] param len
    [5] RSSI    [6][7] PC  [8..19] EPC (12 bytes)  [20][21] CRC  [22] checksum  [23] end

**The upstream code read these one byte late** — RSSI from `[6]`, EPC from `[9]`. Fixed 2026-09-08.
This mattered more than it looks: `[6]` is the PC code's MSB, a constant `0x30` on standard tags, so
**RSSI read as a constant**. Since RSSI arbitration is what distinguishes the two tags on a beam
(decision 4), that bug would have looked like "RSSI can't separate the tags" rather than like a
parsing error. The EPC was shifted a byte, dropping its leading byte and picking up a CRC byte.

`-DDEBUG` is set in the `R200` env because **every print in the successful-read path sits inside
`#ifdef DEBUG`** — without it a working reader looks completely dead.

### `receiveData()` is length-driven — rewritten 2026-09-08

The upstream version scanned for the `0xDD` frame-end byte. Two failures, both fixed:

1. `0xDD` occurs inside EPC and CRC payloads — roughly 5% of random 12-byte EPCs contain one — and
   truncated the frame mid-EPC. About 1,700 of 33,700 tags would have read intermittently.
2. Its `break` on frame-end left only the inner loop, so the outer `while` span the **full timeout on
   every call**, and bytes arriving in the meantime were appended past the frame end, invalidating an
   otherwise good frame.

It now hunts for the `0xAA` header (which also resynchronises after corruption), reads the fixed
5-byte prefix, then consumes exactly `7 + paramLength` bytes total and returns the moment the frame
is complete. Frames claiming more than `RX_BUFFER_LENGTH` are rejected.

The default timeout dropped from 500 ms to **100 ms**. It now only applies to silence, and 500 ms of
it would blow the 150-300 ms sampling window on a single stray byte. A 24-byte frame at 115200 baud
takes ~2 ms, so 100 ms is ~50x margin. The busy-wait calls `yield()` so it can't starve the idle task.

### Other fixes applied to the vendor code

- `calculateCheckSum()` and `dataIsValid()` indexed using an unvalidated wire length. `CRCpos` was a
  `uint8_t`, so `5 + paramLength` wrapped mod 256; `calculateCheckSum()` could walk up to 64 KB past
  a 64-byte buffer. Both bounds-checked now — the likeliest source of unexplained field crashes.
- `parseReceivedData()` returned nothing on any path (compiler-confirmed UB). Note it always used the
  *correct* EPC offset 8, contradicting the `loop()` path that was actually executing.
- `printHexWord()` used `println` for the MSB, splitting every word across two lines.
- Three `char*` parameters that string literals can't bind to in C++11. Build is now warning-free.

### Verified against real hardware, 2026-09-08

A tag read that was checked byte-for-byte:

    RSSI 0xBA   PC 0x3000   EPC E28069150000402092916188   CRC 0xFF46

- The **Gen2 CRC-16 over PC+EPC computes to 0xFF46**, matching what the tag reported. That match is
  only possible if both fields are read from the correct offsets, so the parse is confirmed.
- PC `0x3000` → bits 15-11 = 6 words = 12-byte EPC, consistent with what was read.
- RSSI `0xBA` = **-70 dBm** as a signed int8. It varies per read, which is itself the proof the
  offset fix works — pre-fix this field printed a constant `0x30`.
- EPCs are factory-serialised and unique, starting `E2` (EPCglobal class ID). Good sanity check for
  alignment: if a read ever comes back missing that leading `E2`, offsets have shifted again.

**The library hardcodes 12-byte EPCs** (`memcpy(uid, &_buffer[8], 12)`) even though the PC word
declares the length. Fine while every tag is 96-bit, but it misparses silently rather than erroring
if that ever changes — relevant if decision 5 goes ahead and EPC length becomes ours to choose.

## Invariants

**A stale QR code is worse than no QR code.** If the OLED keeps showing the previous bay's code after
a failed read, the driver scans it confidently and the system reproduces the original bug with its own
authority behind it. Every displayed QR needs a freshness contract:

- Clear the display when the ultrasonic stops seeing a beam
- Expire after a timeout
- Carry a monotonic sequence number so the cabin rejects stale or out-of-order frames
- Blank screen beats wrong screen, always

## Open items

Roughly in order of how much they can still invalidate the design.

- **Confirm RSSI actually separates two tags on one beam.** The single highest risk in the project:
  decision 4 rests entirely on it, and nothing has tested it yet. Two parts. (a) Does RSSI track
  distance sensibly — sweep one tag from ~10 cm to ~2 m and check it climbs toward -40/-50 close in
  and falls to -80/-90 at range. (b) The real geometry — two tags 1.2 m apart as they sit on a beam,
  antenna at its mounting angle, **attenuator fitted**, read from the fork's actual position. The
  number that matters is the *margin* between the two, because that sets the "margin too thin, display
  nothing" threshold. Bench figures will flatter; multipath in a metal canyon is where this is decided.
- **Verify the QR payload byte-for-byte.** The generated QR must encode exactly what the beam labels
  encode today, because the Zebra -> Stockholm parser expects that format. Scan an existing beam label,
  capture the raw decoded string, match precisely. Cheap now, expensive to find during a pilot.
- **Test whether the RF200 can write EPCs** (gates decision 5). Beyond "does it write", check whether
  the tags ship locked or with an access password set — some pre-encoded stock does, which turns a
  firmware question into a procurement one.
- Log RSSI per tag type at a fixed distance. Anti-metal performance varies widely between products and
  this is a 33,700-unit purchasing decision.
- Reconcile the BOM: it lists a "3.5 inch OLED", the hardware is a 3.2" ILI9341 TFT.

## Next steps

The two units have been brought up independently and neither talks to the other yet.

1. ESP-NOW link between the two dev boards on the bench (`esp_get_mac_address` env prints the MACs
   needed to peer them). Testable now, without the RF200 or the fork hardware.
2. Feed a real EPC from the R200 into the cabin unit over that link and render its QR — the first
   end-to-end path.
3. The lookup table: build the packed format, load it into PSRAM, persist to LittleFS.
4. Firebase sync on the cabin unit, in an idle window.

## Display rendering rules

Learned while bringing up the panel; these are scan-reliability requirements, not preferences.

- **Black modules on white, never inverted.** Most imagers refuse an inverted QR.
- **Keep the 4-module quiet zone.** The most common reason a screen-rendered code fails to scan
  while the same data prints fine on a label.
- **Integer pixels per module.** A fractional scale gives uneven module edges and marginal scans.
- **If PWM-dimming the backlight, run LEDC at >= 20 kHz.** Low-frequency PWM bands a rolling-shutter
  imager and causes intermittent scan failures that look like random flakiness.
- Prefer raising the QR version over lowering ECC — these get scanned off a screen in a dusty cab.

## Build

One PlatformIO env per target or bench test, each isolated by `build_src_filter`. **Every env needs
one** — without it PlatformIO compiles every `.cpp` under `src/` and collides on duplicate
`setup()`/`loop()` at link time.

| Env | Source | State |
|---|---|---|
| `cabin_unit` | `src/cabin_unit/` | Display + QR bring-up working |
| `fork_unit` | `src/fork_unit/` | Builds; boot banner only, no functionality |
| `R200` | `src/R200.cpp` | Reader bring-up, **confirmed reading real tags** |
| `ESP-NOW_test` | `src/ESP-NOW_test.cpp` | Bench test |
| `esp_get_mac_address` | `src/esp_get_mac_address.cpp` | Prints MAC, needed to peer ESP-NOW |

Build or flash a specific one with `-e`, e.g. `pio run -e R200 -t upload && pio device monitor -e R200`.

Two layout traps already hit once each, worth not repeating:

- A `lib/` library must sit in **its own subdirectory** (`lib/R200/R200.{h,cpp}`). Files loose in
  `lib/` are silently ignored — not compiled, not on the include path — and the symptom is an
  unresolved `#include` followed by undefined references at link time.
- `src/fork_unit/` was once created as `src/ fork_unit/` with a leading space, which
  `build_src_filter = +<fork_unit>` silently never matches.
- Don't copy the `cabin_unit` env wholesale when adding a new one. Carrying over its TFT_eSPI
  `lib_deps` plus `-DUSER_SETUP_LOADED` while pointing `-include` at a different (empty) config
  suppresses TFT_eSPI's own setup and leaves every `TFT_*` symbol undefined. The fork unit has no
  display and needs neither.

### R200 wiring

`rfid.begin(&Serial2, 115200, 18, 17)` → **GPIO18 = RX** (to the reader's TX), **GPIO17 = TX** (to
the reader's RX), common ground. Both are clear of the PSRAM range (33-37) and the USB pins (19/20).
If the module info string comes back garbled or absent, suspect baud or a swapped TX/RX before
suspecting the tags.

**TFT_eSPI configuration is not in a `User_Setup.h`.** It lives in `src/cabin_unit/config.h` and is
delivered via two build flags that must stay together:

    -DUSER_SETUP_LOADED=1
    -include "${PROJECT_DIR}/src/cabin_unit/config.h"

TFT_eSPI builds as its own library, so a header in `src/` is invisible to it — without the
`-include` it silently uses its bundled setup for a plain ESP32 on different pins, and the symptom
is a blank panel that looks exactly like a wiring fault. Verified as of 2026-09-03 that both flags
reach TFT_eSPI's own compilation unit, with the spaces in the project path correctly quoted.

### TFT_eSPI on ESP32-S3 requires `USE_FSPI_PORT`

A library bug, not a wiring fault, and it costs an afternoon if rediscovered. Without that define
the firmware panics inside `tft.init()`:

    Guru Meditation Error: Core 1 panic'ed (StoreProhibited)
    EXCVADDR: 0x00000010 ... in TFT_eSPI::begin_tft_write()

The chain: TFT_eSPI falls back to `#define SPI_PORT VSPI`; the S3 has no VSPI so the library
aliases `VSPI -> FSPI`; the Arduino core defines `FSPI` as `0`; and `soc.h` has
`REG_SPI_BASE(i)` return **0 for any i < 2** (the S3 only has SPI2 and SPI3). Every register
pointer becomes a raw offset from null — `SPI_USER_REG(0)` = `0 + 0x10` = `0x10` — and the first
bus write stores to `0x10`.

`USE_FSPI_PORT` sets `SPI_PORT` to a literal `2`, so `REG_SPI_BASE(2)` = SPI2, the peripheral the
IO_MUX pins belong to.

### Backlight is hardwired to 3V3

`TFT_BL` is deliberately left undefined so TFT_eSPI doesn't toggle an unconnected GPIO. Brightness
is therefore fixed at full — a glare problem for a cab at night, and constant power draw. Regaining
control needs a transistor on a spare GPIO (16 is free); never drive the LED rail straight from a
pin, it can pull 60-100 mA against the S3's 40 mA per-pin limit.

## Status as of 2026-09-08

- **Cabin unit**: builds and runs. Colour self-test, then cycles sample racking locations as QR codes
  on the ILI9341. Sample payloads in `main.cpp` are placeholders, not the real label format.
- **R200 reader**: builds and reads real tags, frame parsing verified against the Gen2 CRC.
- **Not started**: ESP-NOW link, lookup table, LittleFS persistence, Firebase sync, ultrasonic gating,
  RSSI arbitration, staleness enforcement. `src/fork_unit/` is an empty placeholder.

`board_upload.flash_size` and `board_build.partitions` are overridden to 16 MB because the board
profile defaults to an 8 MB partition table, which would strand half the chip.
