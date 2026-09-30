// Cabin unit.
//
// Receives read reports from the fork unit over ESP-NOW, decides whether one
// tag clearly wins, and shows it as a QR code. All of that decision's tuning
// lives here rather than on the fork, because this unit can be reflashed
// without unbolting anything (decision 2).
//
// Until the lookup table exists, the QR encodes the raw EPC in hex. That is a
// bench placeholder only: the real payload is the location string the beam
// labels encode today, byte for byte (CLAUDE.md "Open items").

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <qrcode.h>

#include "truck_link.h"
#include "truck_pairing.h"

using truck_link::kEpcLength;

// ---- Read confirmation policy ---------------------------------------------
//
// A tag is shown only if, pooled over the last kEvidenceMs of reports:
//   1. it was read at least kMinHits times (kKeepHits if it is already up), and
//   2. its mean RSSI beats every other contender by at least kMinMarginDb.
// Hit count alone cannot choose between the two tags on one beam -- both are
// usually in range and both will clear any hit threshold. The margin is what
// actually decides; the hit count is what makes the margin trustworthy.

// How much recent evidence to pool. Inside decision 4's 150-300 ms band. The
// fork reports every 100 ms, so this is about three reports.
static constexpr uint32_t kEvidenceMs = 300;

// Minimum reads of the winner within the evidence window, to put a code up.
static constexpr uint16_t kMinHits = 5;

// Minimum reads to keep a code that is already up. Hysteresis: the bar to
// start showing a tag is higher than the bar to keep showing it, so a tag
// that fades as the carriage moves (read rate drops with signal, e.g. 7/9
// polls at -49 dBm but 3/17 at -70 dBm on the bench) doesn't flicker off
// while it is plainly still the tag being read. The margin rule still
// applies, so a rival closing in blanks it regardless.
static constexpr uint16_t kKeepHits = 2;

// Tags read fewer times than this are treated as noise, not as contenders.
// Otherwise a single stray reflection off a neighbour would blank a solid read.
// Must not exceed kKeepHits, or a kept tag could never qualify as leader.
static constexpr uint16_t kMinContenderHits = 2;

// PLACEHOLDER. This is the number the open item "confirm RSSI separates two
// tags on one beam" exists to measure -- with the attenuator fitted, at the
// antenna's mounting angle, in real racking. 6 dB is a bench starting point.
static constexpr int16_t kMinMarginDb = 6;

// A shown code is blanked if nothing has re-confirmed it for this long.
static constexpr uint32_t kHoldMs = 600;

// No report at all for this long means the fork unit or the link is down.
static constexpr uint32_t kLinkTimeoutMs = 1000;

// ---- Display ----------------------------------------------------------------

static TFT_eSPI tft = TFT_eSPI();

// Version 3 is 29x29 modules. The 24-char EPC encodes in alphanumeric mode with
// plenty of room at ECC_MEDIUM. If the real location payload turns out longer,
// raise the version (4 = 33x33) rather than dropping the error correction:
// these get scanned off a screen in a dusty cab, so ECC headroom is worth more
// than size.
static constexpr uint8_t kQrVersion = 3;

// The QR spec requires a 4-module light border. Scanners genuinely fail
// without it, and it is the most common reason a screen-rendered code is
// unreadable while the same data prints fine on a label.
static constexpr int16_t kQuietModules = 4;

// Two text lines under the QR: the payload, then the read statistics.
static constexpr int16_t kLabelHeight = 56;

enum class Screen : uint8_t
{
  None,
  NoLink,
  NoTag,
  Ambiguous,
  Tag,
};

static Screen screen = Screen::None;
static uint8_t shownEpc[kEpcLength];

// Black modules on a white ground, never inverted -- imagers expect dark-on-light
// and a large fraction will refuse an inverted code.
static void drawLocationQr(const char* text)
{
  QRCode qr;
  uint8_t buffer[qrcode_getBufferSize(kQrVersion)];

  if (qrcode_initText(&qr, buffer, kQrVersion, ECC_MEDIUM, text) < 0)
  {
    Serial.printf("QR encode FAILED for \"%s\" (too long for version %u?)\n", text, kQrVersion);
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
  tft.drawString(text, tft.width() / 2, tft.height() - kLabelHeight + 14, font);

  Serial.printf("drew \"%s\"  %dx%d modules @ %dpx = %dpx\n", text, modules, modules, scale, rendered);
}

// Redrawn on every confirmation while the same code stays up, so the bench can
// watch RSSI and margin live. Only this strip is touched, never the QR.
static void drawReadStats(const char* line)
{
  const int16_t y = tft.height() - 14;
  tft.fillRect(0, y - 9, tft.width(), 18, TFT_WHITE);
  tft.setTextColor(TFT_DARKGREY, TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(line, tft.width() / 2, y, 2);
}

// Nothing scannable on screen. The message is for diagnosis; it is text, not a
// code, so it cannot be scanned into WMS by mistake.
static void showBlank(Screen reason)
{
  if (screen == reason)
  {
    return;
  }

  const char* message = "";
  switch (reason)
  {
    case Screen::NoLink:
      message = "FORK UNIT OFFLINE";
      break;
    case Screen::NoTag:
      message = "NO TAG";
      break;
    case Screen::Ambiguous:
      message = "AMBIGUOUS READ";
      break;
    default:
      break;
  }

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(message, tft.width() / 2, tft.height() / 2, 2);

  screen = reason;
  Serial.printf("screen: %s\n", message);
}

// Distinguishes "SPI is dead" from "panel works but colours are off" before
// QR complexity enters the picture. A blank screen here means wiring or the
// TFT_eSPI config never loaded; wrong colours mean driver or RGB order.
static void panelSelfTest()
{
  struct
  {
    uint16_t colour;
    const char* name;
  } steps[] = {
    { TFT_RED, "RED" },
    { TFT_GREEN, "GREEN" },
    { TFT_BLUE, "BLUE" },
    { TFT_WHITE, "WHITE" },
  };

  for (auto& step : steps)
  {
    Serial.printf("self-test: %s\n", step.name);
    tft.fillScreen(step.colour);
    tft.setTextColor(TFT_BLACK, step.colour);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(step.name, tft.width() / 2, tft.height() / 2, 4);
    delay(500);
  }
}

// ---- Receiving --------------------------------------------------------------

struct Received
{
  truck_link::ReadReport report;
  uint32_t rxMs;
};

static QueueHandle_t inbox;
static volatile uint32_t rejectedFrames = 0;

// Runs in the WiFi task, so it only validates and queues. All drawing happens
// in loop().
static void onReceive(const uint8_t* mac, const uint8_t* data, int len)
{
  // Unicast frames meant for other trucks' cabins never reach this callback,
  // but a broadcast or a mis-paired unit would. Another truck's location must
  // never reach this screen.
  if (memcmp(mac, truck_pairing::kForkMac, 6) != 0 || len < (int)truck_link::kHeaderSize)
  {
    rejectedFrames++;
    return;
  }

  Received rx = {};
  memcpy(&rx.report, data, truck_link::kHeaderSize);
  if (rx.report.version != truck_link::kProtocolVersion || rx.report.tagCount > truck_link::kMaxTags ||
      len != (int)truck_link::reportSize(rx.report.tagCount))
  {
    rejectedFrames++;
    return;
  }

  memcpy(&rx.report, data, len);
  rx.rxMs = millis();
  if (xQueueSend(inbox, &rx, 0) != pdTRUE)
  {
    rejectedFrames++;
  }
}

// Freshness: reports must arrive with strictly increasing seq. A new bootId
// means the fork restarted and its seq began again at 1, so tracking restarts
// instead of rejecting everything from the new boot as stale.
static bool haveSession = false;
static uint32_t sessionBootId = 0;
static uint32_t lastSeq = 0;
static uint32_t staleFrames = 0;

// Recent accepted reports, oldest first.
static constexpr size_t kHistory = 8;
static Received history[kHistory];
static size_t historyCount = 0;

static bool isFresh(const truck_link::ReadReport& report)
{
  if (!haveSession || report.bootId != sessionBootId)
  {
    haveSession = true;
    sessionBootId = report.bootId;
    lastSeq = report.seq;
    historyCount = 0;  // evidence from the previous boot is not evidence now
    Serial.printf("fork unit session %08lx\n", (unsigned long)report.bootId);
    return true;
  }
  if (report.seq <= lastSeq)
  {
    return false;
  }
  lastSeq = report.seq;
  return true;
}

static void remember(const Received& rx)
{
  if (historyCount == kHistory)
  {
    memmove(&history[0], &history[1], sizeof(history[0]) * (kHistory - 1));
    historyCount--;
  }
  history[historyCount++] = rx;
}

// ---- Arbitration ------------------------------------------------------------

struct Candidate
{
  uint8_t epc[kEpcLength];
  uint16_t hits;
  int32_t rssiSum;  // weighted by hits, so the pooled mean is per read, not per report

  int16_t rssi() const
  {
    return (int16_t)(rssiSum / hits);
  }
};

static constexpr size_t kMaxCandidates = 16;

// Pool every report received within the evidence window, per EPC.
static size_t gatherEvidence(uint32_t now, Candidate* out, uint16_t& polls)
{
  size_t count = 0;
  polls = 0;

  for (size_t r = 0; r < historyCount; r++)
  {
    if (now - history[r].rxMs >= kEvidenceMs)
    {
      continue;
    }
    const truck_link::ReadReport& report = history[r].report;
    polls += report.polls;

    for (uint8_t t = 0; t < report.tagCount; t++)
    {
      const truck_link::TagReport& tag = report.tags[t];
      size_t i = 0;
      while (i < count && memcmp(out[i].epc, tag.epc, kEpcLength) != 0)
      {
        i++;
      }
      if (i == count)
      {
        if (count == kMaxCandidates)
        {
          continue;
        }
        memcpy(out[count].epc, tag.epc, kEpcLength);
        out[count].hits = 0;
        out[count].rssiSum = 0;
        count++;
      }
      out[i].hits += tag.hits;
      out[i].rssiSum += (int32_t)tag.rssi * tag.hits;
    }
  }
  return count;
}

enum class Verdict : uint8_t
{
  Confirmed,     // show the leader
  Kept,          // leader is the code already up, re-read enough to keep it
  Ambiguous,     // a contender is too close to call -- blank now
  Insufficient,  // not enough evidence either way
};

struct Decision
{
  Verdict verdict;
  const Candidate* leader;    // strongest contender, if any
  const Candidate* runnerUp;  // next strongest, if any
};

// `shown` is the EPC currently on screen, or nullptr if the screen is blank.
static Decision arbitrate(const Candidate* cands, size_t count, const uint8_t* shown)
{
  Decision d = { Verdict::Insufficient, nullptr, nullptr };

  for (size_t i = 0; i < count; i++)
  {
    const Candidate* c = &cands[i];
    if (c->hits < kMinContenderHits)
    {
      continue;
    }
    if (!d.leader || c->rssi() > d.leader->rssi())
    {
      d.runnerUp = d.leader;
      d.leader = c;
    }
    else if (!d.runnerUp || c->rssi() > d.runnerUp->rssi())
    {
      d.runnerUp = c;
    }
  }

  if (!d.leader)
  {
    return d;
  }
  if (d.runnerUp && d.leader->rssi() - d.runnerUp->rssi() < kMinMarginDb)
  {
    d.verdict = Verdict::Ambiguous;
  }
  else if (d.leader->hits >= kMinHits)
  {
    d.verdict = Verdict::Confirmed;
  }
  else if (shown && d.leader->hits >= kKeepHits && memcmp(d.leader->epc, shown, kEpcLength) == 0)
  {
    d.verdict = Verdict::Kept;
  }
  // Otherwise a clear leader that hasn't been read often enough yet.
  return d;
}

static const char* verdictName(Verdict v)
{
  switch (v)
  {
    case Verdict::Confirmed:
      return "CONFIRMED";
    case Verdict::Kept:
      return "KEPT";
    case Verdict::Ambiguous:
      return "AMBIGUOUS";
    default:
      return "insufficient";
  }
}

static void logCandidate(const Candidate* c)
{
  char hex[kEpcLength * 2 + 1];
  truck_link::epcToHex(c->epc, hex);
  Serial.printf(" | %s %d dBm x%u", hex, c->rssi(), c->hits);
}

static uint32_t lastReportMs = 0;
static uint32_t lastConfirmMs = 0;

static void decide(uint32_t now)
{
  Candidate cands[kMaxCandidates];
  uint16_t polls = 0;
  const size_t count = gatherEvidence(now, cands, polls);
  const Decision d = arbitrate(cands, count, screen == Screen::Tag ? shownEpc : nullptr);

  if (count > 0)
  {
    Serial.printf("%-12s polls %u", verdictName(d.verdict), polls);
    for (size_t i = 0; i < count; i++)
    {
      logCandidate(&cands[i]);
    }
    Serial.println();
  }

  switch (d.verdict)
  {
    case Verdict::Confirmed:
    case Verdict::Kept: {
      char hex[kEpcLength * 2 + 1];
      truck_link::epcToHex(d.leader->epc, hex);
      if (screen != Screen::Tag || memcmp(shownEpc, d.leader->epc, kEpcLength) != 0)
      {
        drawLocationQr(hex);
        memcpy(shownEpc, d.leader->epc, kEpcLength);
        screen = Screen::Tag;
      }

      char stats[40];
      if (d.runnerUp)
      {
        snprintf(stats, sizeof(stats), "%d dBm  x%u  margin %d dB", d.leader->rssi(), d.leader->hits,
                 d.leader->rssi() - d.runnerUp->rssi());
      }
      else
      {
        snprintf(stats, sizeof(stats), "%d dBm  x%u  no contender", d.leader->rssi(), d.leader->hits);
      }
      drawReadStats(stats);
      lastConfirmMs = now;
      break;
    }

    case Verdict::Ambiguous:
      showBlank(Screen::Ambiguous);
      break;

    case Verdict::Insufficient:
      // Not enough to show anything, but not evidence against the code on
      // screen either -- unless the strongest tag is now a different one. The
      // hold timeout covers the rest.
      if (screen == Screen::Tag)
      {
        if (d.leader && memcmp(d.leader->epc, shownEpc, kEpcLength) != 0)
        {
          showBlank(Screen::Ambiguous);
        }
      }
      else
      {
        showBlank(Screen::NoTag);
      }
      break;
  }
}

// The stale-QR invariant: a code comes down on its own unless it keeps being
// re-confirmed, and everything comes down if the fork unit goes quiet.
static void enforceTimeouts(uint32_t now)
{
  if (!haveSession || now - lastReportMs > kLinkTimeoutMs)
  {
    showBlank(Screen::NoLink);
  }
  else if (screen == Screen::Tag && now - lastConfirmMs > kHoldMs)
  {
    showBlank(Screen::NoTag);
  }
}

// ---- Arduino ------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(2000);  // let the host attach before the first print

  Serial.println();
  Serial.println("=== cabin unit ===");
  Serial.printf("psram: %lu bytes (%s)\n", (unsigned long)ESP.getPsramSize(), psramFound() ? "detected" : "NOT FOUND");
  Serial.printf("panel: %dx%d @ %d Hz SPI\n", TFT_WIDTH, TFT_HEIGHT, SPI_FREQUENCY);

  tft.init();
  tft.setRotation(0);  // portrait; QR above, text below

  // No backlight control here on purpose: the LED rail is hardwired to 3V3.
  // See config.h if that ever moves onto a GPIO.

  panelSelfTest();
  showBlank(Screen::NoLink);

  // Before ESP-NOW starts, so the receive callback never sees a null queue.
  inbox = xQueueCreate(8, sizeof(Received));

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(truck_pairing::kChannel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init FAILED -- nothing will be received");
    return;
  }
  esp_now_register_recv_cb(onReceive);

  Serial.printf("listening for fork unit on channel %u\n", truck_pairing::kChannel);
}

void loop()
{
  Received rx;
  bool received = false;

  // Wait briefly for the first report, then drain whatever else has queued.
  TickType_t wait = pdMS_TO_TICKS(20);
  while (xQueueReceive(inbox, &rx, wait) == pdTRUE)
  {
    wait = 0;
    if (!isFresh(rx.report))
    {
      staleFrames++;
      continue;
    }
    remember(rx);
    lastReportMs = rx.rxMs;
    received = true;
  }

  const uint32_t now = millis();
  if (received)
  {
    decide(now);
  }
  enforceTimeouts(now);

  static uint32_t loggedRejected = 0;
  static uint32_t loggedStale = 0;
  if (rejectedFrames != loggedRejected || staleFrames != loggedStale)
  {
    loggedRejected = rejectedFrames;
    loggedStale = staleFrames;
    Serial.printf("dropped: %lu rejected (sender, version, length or queue full), %lu stale\n",
                  (unsigned long)loggedRejected, (unsigned long)loggedStale);
  }
}
