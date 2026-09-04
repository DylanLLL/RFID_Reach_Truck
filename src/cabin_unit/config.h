// TFT_eSPI setup for the cabin unit's 3.2" ILI9341 panel (240x320, no touch).
//
// This file is force-included into every translation unit via -include in
// platformio.ini, alongside -DUSER_SETUP_LOADED. Both are required: TFT_eSPI
// builds as a separate library and would otherwise use its own User_Setup.h.

#pragma once

#define ILI9341_DRIVER
#define TFT_WIDTH 240
#define TFT_HEIGHT 320

// REQUIRED on ESP32-S3 -- without it the firmware crashes in tft.init().
//
// TFT_eSPI's fallback is #define SPI_PORT VSPI. The S3 has no VSPI, so the
// library aliases VSPI -> FSPI, and the Arduino core defines FSPI as 0. But
// soc.h has REG_SPI_BASE(i) return 0 for any i < 2 (the S3 only has SPI2 and
// SPI3), so every register pointer becomes a raw offset from null:
// SPI_USER_REG(0) = 0 + 0x10 = 0x10, and the first bus write panics with
// StoreProhibited at 0x10.
//
// USE_FSPI_PORT sets SPI_PORT to a literal 2 -> REG_SPI_BASE(2) = SPI2, which
// is the peripheral our IO_MUX pins below belong to.
#define USE_FSPI_PORT

// SPI pins are the ESP32-S3 native IO_MUX set for FSPI/SPI2 (CLK=12, MOSI=11,
// MISO=13, CS0=10). Keeping to these routes SPI directly through IO_MUX rather
// than the GPIO matrix, which is what allows a clean 40 MHz. Do not move them
// to arbitrary GPIOs without expecting to drop the clock.
#define TFT_MISO -1  // unconnected; -1 frees GPIO13 and avoids sampling a floating pin
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS 10
#define TFT_DC 14   // FSPIWP, free because we run 1-bit SPI
#define TFT_RST 21  // kept under GPIO control so the panel can be hard-reset after a vibration glitch

// Backlight is hardwired to the ESP32's 3V3 pin, not driven by a GPIO, so
// TFT_BL is deliberately left undefined -- TFT_eSPI then skips backlight
// handling entirely rather than toggling an unconnected pin.
//
// Consequence: brightness is fixed at full. For a cab at night that is a glare
// problem, and it burns power continuously. Regaining control means a MOSFET or
// transistor on a spare GPIO (16 is still free) -- do not drive the LED rail
// straight from a pin, it can pull 60-100 mA against the S3's 40 mA limit.
// If that is added, PWM it with LEDC at >= 20 kHz: low-frequency PWM bands a
// rolling-shutter imager and causes intermittent scan failures.
// #define TFT_BL 16
// #define TFT_BACKLIGHT_ON HIGH

// Fonts. Without at least one LOAD_*, drawString() has nothing to render.
#define LOAD_GLCD   // font 1, 8px
#define LOAD_FONT2  // 16px
#define LOAD_FONT4  // 26px, used for the location string under the QR
#define LOAD_GFXFF

// ILI9341 is specified around 10 MHz for writes but runs 40 MHz reliably on
// IO_MUX pins. On a vehicle with dupont jumpers, 40 is the sensible ceiling --
// if the display glitches under vibration, step down to 27000000 rather than up.
#define SPI_FREQUENCY 40000000
#define SPI_READ_FREQUENCY 20000000
