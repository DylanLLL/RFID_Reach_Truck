// Cabin unit -- display bring-up test.
//
// Proves the ILI9341 panel and the QR rendering path before any ESP-NOW or
// lookup-table work goes in. Runs a colour self-test, then cycles through
// sample racking locations as scannable QR codes.

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <qrcode.h>

static TFT_eSPI tft = TFT_eSPI();

// Placeholder payloads. These MUST eventually match what the existing beam
// labels encode, byte for byte -- the Zebra -> Stockholm parser expects that
// exact format. Scan a real beam label and capture the raw string before this
// goes anywhere near a pilot. See CLAUDE.md "Open items".
static const char *const kSampleLocations[] = {
    "MRD-R12-B08-L03-A",
    "MRD-R12-B08-L03-B",
    "MRD-R01-B01-L01-A",
    "MRD-R47-B22-L05-B",
};
static const size_t kSampleCount =
    sizeof(kSampleLocations) / sizeof(kSampleLocations[0]);

// Version 3 is 29x29 modules and holds 32 bytes at ECC_MEDIUM -- ample for a
// ~17 char location. If the real payload turns out longer, raise the version
// (4 = 33x33/46 bytes) rather than dropping the error correction: these get
// scanned off a screen in a dusty cab, so ECC headroom is worth more than size.
static constexpr uint8_t kQrVersion = 3;

// The QR spec requires a 4-module light border. Scanners genuinely fail
// without it, and it is the most common reason a screen-rendered code is
// unreadable while the same data prints fine on a label.
static constexpr int16_t kQuietModules = 4;

static constexpr int16_t kLabelHeight = 56;

// Black modules on a white ground, never inverted -- imagers expect dark-on-light
// and a large fraction will refuse an inverted code.
static void drawLocationQr(const char *text)
{
  QRCode qr;
  uint8_t buffer[qrcode_getBufferSize(kQrVersion)];

  if (qrcode_initText(&qr, buffer, kQrVersion, ECC_MEDIUM, text) < 0)
  {
    Serial.printf("QR encode FAILED for \"%s\" (too long for version %u?)\n",
                  text, kQrVersion);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("QR ENCODE FAILED", tft.width() / 2, tft.height() / 2, 2);
    return;
  }

  const int16_t modules = qr.size;
  const int16_t total = modules + 2 * kQuietModules;

  const int16_t availW = tft.width();
  const int16_t availH = tft.height() - kLabelHeight;
  const int16_t avail = (availW < availH) ? availW : availH;

  // Integer scale only. A fractional module size means uneven module edges,
  // which is exactly what makes a screen QR marginal to scan.
  const int16_t scale = avail / total;
  const int16_t rendered = total * scale;

  const int16_t originX = (availW - rendered) / 2;
  const int16_t originY = (availH - rendered) / 2;
  const int16_t qrX = originX + kQuietModules * scale;
  const int16_t qrY = originY + kQuietModules * scale;

  tft.fillScreen(TFT_WHITE);

  for (int16_t y = 0; y < modules; y++)
  {
    for (int16_t x = 0; x < modules; x++)
    {
      if (qrcode_getModule(&qr, x, y))
      {
        tft.fillRect(qrX + x * scale, qrY + y * scale, scale, scale, TFT_BLACK);
      }
    }
  }

  // Human-readable fallback so the driver can still read the location aloud
  // if the scan fails. Drop to the smaller font if the string overruns.
  const uint8_t font = (tft.textWidth(text, 4) > tft.width() - 8) ? 2 : 4;
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text, tft.width() / 2, tft.height() - kLabelHeight / 2, font);

  Serial.printf("drew \"%s\"  %dx%d modules @ %dpx = %dpx\n", text, modules,
                modules, scale, rendered);
}

// Distinguishes "SPI is dead" from "panel works but colours are off" before
// QR complexity enters the picture. A blank screen here means wiring or the
// TFT_eSPI config never loaded; wrong colours mean driver or RGB order.
static void panelSelfTest()
{
  struct
  {
    uint16_t colour;
    const char *name;
  } steps[] = {
      {TFT_RED, "RED"},
      {TFT_GREEN, "GREEN"},
      {TFT_BLUE, "BLUE"},
      {TFT_WHITE, "WHITE"},
  };

  for (auto &step : steps)
  {
    Serial.printf("self-test: %s\n", step.name);
    tft.fillScreen(step.colour);
    tft.setTextColor(TFT_BLACK, step.colour);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(step.name, tft.width() / 2, tft.height() / 2, 4);
    delay(500);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(2000);  // let the host attach before the first print

  Serial.println();
  Serial.println("=== cabin unit: display bring-up ===");
  Serial.printf("psram: %lu bytes (%s)\n", (unsigned long)ESP.getPsramSize(),
                psramFound() ? "detected" : "NOT FOUND");
  Serial.printf("panel: %dx%d @ %d Hz SPI\n", TFT_WIDTH, TFT_HEIGHT,
                SPI_FREQUENCY);

  tft.init();
  tft.setRotation(0);  // portrait; QR above, location text below

  // No backlight control here on purpose: the LED rail is hardwired to 3V3.
  // See config.h if that ever moves onto a GPIO.

  panelSelfTest();
  Serial.println("self-test done, cycling sample locations");
}

void loop()
{
  static size_t index = 0;

  drawLocationQr(kSampleLocations[index]);
  index = (index + 1) % kSampleCount;

  delay(3000);
}
