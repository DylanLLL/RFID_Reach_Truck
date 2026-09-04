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

## Invariants

**A stale QR code is worse than no QR code.** If the OLED keeps showing the previous bay's code after
a failed read, the driver scans it confidently and the system reproduces the original bug with its own
authority behind it. Every displayed QR needs a freshness contract:

- Clear the display when the ultrasonic stops seeing a beam
- Expire after a timeout
- Carry a monotonic sequence number so the cabin rejects stale or out-of-order frames
- Blank screen beats wrong screen, always

## Open items

- **Verify the QR payload byte-for-byte.** The generated QR must encode exactly what the beam labels
  encode today, because the Zebra -> Stockholm parser expects that format. Scan an existing beam label,
  capture the raw decoded string, match precisely. Cheap now, expensive to find during a pilot.
- Test whether the RF200 can write EPCs (gates decision 5).
- Confirm real-world RSSI separation between the two tags on one beam, with the attenuator fitted and
  the antenna at its mounting angle.

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

Two PlatformIO envs in one project, selected by `build_src_filter`:

- `cabin_unit` -> `src/cabin_unit/` — active
- fork unit env is currently commented out in `platformio.ini`; `src/main.cpp` is the leftover
  boilerplate it used to build. When re-enabling it, name it `fork_unit`, point it at
  `src/fork_unit/`, and give it a `build_src_filter` — without one it compiles every `main.cpp`
  under `src/` and collides on duplicate `setup()`/`loop()` at link time.

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

Cabin unit currently builds and runs a display bring-up test: colour self-test, then cycles sample
racking locations as QR codes. No ESP-NOW, lookup table or Firebase sync yet. Fork unit is unwritten.

`board_upload.flash_size` and `board_build.partitions` are overridden to 16 MB because the board
profile defaults to an 8 MB partition table, which would strand half the chip.
