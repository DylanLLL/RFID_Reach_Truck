// Fork unit.
//
// The battery-powered sensor on the fork carriage. Per decision 2 in CLAUDE.md
// it stays deliberately dumb: it never associates to WiFi, holds no lookup
// table, and makes no decisions. It runs RFID inventory rounds back to back,
// tallies hits and RSSI per EPC, and every kReportIntervalMs ships the tally to
// the cabin unit over ESP-NOW -- empty tallies included, so the cabin can tell
// "no tag in range" from "fork unit gone quiet".
//
// Every judgement -- how many hits make a confident read, what RSSI margin
// separates two tags on one beam, what to display -- lives on the cabin unit,
// because that one can be reflashed without unbolting anything.
//
// Not here yet: ultrasonic gating. Until it is, the reader runs whenever the
// unit is powered, which is fine on a bench and wrong for battery life.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "R200.h"
#include "truck_link.h"
#include "truck_pairing.h"

// Reader UART, as proven in src/R200.cpp: GPIO18 = RX (to the reader's TX),
// GPIO17 = TX (to the reader's RX).
static constexpr uint8_t kReaderRxPin = 18;
static constexpr uint8_t kReaderTxPin = 17;

// Transport granularity, not the decision window. The cabin chooses how many
// reports' worth of evidence to weigh, so that window can be tuned without
// reflashing this unit.
static constexpr uint32_t kReportIntervalMs = 100;

// Inventory pacing. A single poll is answered by one frame per tag in range,
// or by a single "no tag" error, and there is no end-of-round marker. So a
// round counts as finished on the "no tag" frame, or once the line has been
// quiet this long after the last frame (a 24-byte frame is ~2 ms at 115200).
static constexpr uint32_t kRoundQuietMs = 10;

// A round with no answer at all is abandoned after this. If every round times
// out, the reader is unpowered, miswired, or on the wrong baud.
static constexpr uint32_t kRoundTimeoutMs = 100;

static R200 rfid;

struct Tally
{
  uint8_t epc[truck_link::kEpcLength];
  int32_t rssiSum;
  uint8_t hits;
};

// The current report interval's evidence.
static Tally tallies[truck_link::kMaxTags];
static uint8_t tallyCount = 0;
static uint8_t droppedReads = 0;
static uint8_t pollCount = 0;

static uint32_t bootId = 0;
static uint32_t seq = 0;
static uint32_t intervalStartMs = 0;

// Round pacing state.
static bool roundOpen = false;
static bool roundAnswered = false;
static uint32_t roundStartMs = 0;
static uint32_t lastFrameMs = 0;

// Per-second health counters, for the serial log.
static uint32_t healthStartMs = 0;
static uint32_t healthPolls = 0;
static uint32_t healthTimeouts = 0;
static volatile uint32_t sendFailures = 0;

static void onSent(const uint8_t* mac, esp_now_send_status_t status)
{
  // Runs in the WiFi task. Delivery failure means the cabin never ACKed --
  // powered off, out of range, or on another channel.
  if (status != ESP_NOW_SEND_SUCCESS)
  {
    sendFailures++;
  }
}

static void recordRead(const uint8_t* epc, int8_t rssi)
{
  for (uint8_t i = 0; i < tallyCount; i++)
  {
    if (memcmp(tallies[i].epc, epc, truck_link::kEpcLength) == 0)
    {
      if (tallies[i].hits < UINT8_MAX)
      {
        tallies[i].hits++;
        tallies[i].rssiSum += rssi;
      }
      return;
    }
  }

  if (tallyCount < truck_link::kMaxTags)
  {
    Tally& t = tallies[tallyCount++];
    memcpy(t.epc, epc, truck_link::kEpcLength);
    t.rssiSum = rssi;
    t.hits = 1;
  }
  else if (droppedReads < UINT8_MAX)
  {
    droppedReads++;
  }
}

static void sendReport()
{
  truck_link::ReadReport report = {};
  report.version = truck_link::kProtocolVersion;
  report.tagCount = tallyCount;
  report.polls = pollCount;
  report.dropped = droppedReads;
  report.bootId = bootId;
  report.seq = ++seq;

  for (uint8_t i = 0; i < tallyCount; i++)
  {
    memcpy(report.tags[i].epc, tallies[i].epc, truck_link::kEpcLength);
    report.tags[i].rssi = (int8_t)(tallies[i].rssiSum / tallies[i].hits);
    report.tags[i].hits = tallies[i].hits;
  }

  const esp_err_t err =
    esp_now_send(truck_pairing::kCabinMac, (const uint8_t*)&report, truck_link::reportSize(tallyCount));

  // Only reports that saw something are logged, to keep the monitor readable.
  // The once-a-second health line proves the unit is alive in between.
  if (tallyCount > 0 || err != ESP_OK)
  {
    Serial.printf("#%lu polls %u", (unsigned long)report.seq, pollCount);
    for (uint8_t i = 0; i < tallyCount; i++)
    {
      char hex[truck_link::kEpcLength * 2 + 1];
      truck_link::epcToHex(report.tags[i].epc, hex);
      Serial.printf(" | %s %d dBm x%u", hex, report.tags[i].rssi, report.tags[i].hits);
    }
    if (droppedReads > 0)
    {
      Serial.printf(" | %u reads dropped, table full", droppedReads);
    }
    if (err != ESP_OK)
    {
      Serial.printf(" | SEND ERROR %s", esp_err_to_name(err));
    }
    Serial.println();
  }

  tallyCount = 0;
  droppedReads = 0;
  pollCount = 0;
}

static void startRound(uint32_t now)
{
  rfid.poll();
  roundOpen = true;
  roundAnswered = false;
  roundStartMs = now;
  if (pollCount < UINT8_MAX)
  {
    pollCount++;
  }
  healthPolls++;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);  // let the host attach before the first print

  Serial.println();
  Serial.println("=== fork unit ===");

  // ESP-NOW needs the radio up in STA mode. The fork never associates to an AP,
  // so nothing moves the channel away from the one pinned here.
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(truck_pairing::kChannel, WIFI_SECOND_CHAN_NONE);

  // After WiFi.mode(): the hardware RNG is only truly random with the radio on.
  bootId = esp_random();

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init FAILED -- every report will fail to send");
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, truck_pairing::kCabinMac, sizeof(peer.peer_addr));
  peer.channel = truck_pairing::kChannel;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
  {
    Serial.println("adding the cabin unit as ESP-NOW peer FAILED");
  }

  Serial.printf("boot id %08lx, reporting to cabin every %lu ms on channel %u\n", (unsigned long)bootId,
                (unsigned long)kReportIntervalMs, truck_pairing::kChannel);

  rfid.begin(&Serial2, 115200, kReaderRxPin, kReaderTxPin);

  // The reply is printed by rfid.loop(). Garbled or missing means baud or a
  // swapped TX/RX, not the tags.
  rfid.dumpModuleInfo();

  intervalStartMs = healthStartMs = millis();
}

void loop()
{
  // Drain everything the reader has sent so far.
  for (;;)
  {
    const R200::R200_Event event = rfid.loop();
    if (event == R200::EVT_None)
    {
      break;
    }

    lastFrameMs = millis();
    roundAnswered = true;

    if (event == R200::EVT_TagRead)
    {
      recordRead(rfid.uid, rfid.rssi);
    }
    else if (event == R200::EVT_NoTag)
    {
      // Nothing in range: the round is definitely over, no need to wait out
      // the quiet gap.
      roundOpen = false;
    }
  }

  const uint32_t now = millis();

  if (roundOpen && !roundAnswered && now - roundStartMs >= kRoundTimeoutMs)
  {
    healthTimeouts++;
    roundOpen = false;
  }
  if (roundOpen && roundAnswered && now - lastFrameMs >= kRoundQuietMs)
  {
    roundOpen = false;
  }
  if (!roundOpen)
  {
    startRound(now);
  }

  if (now - intervalStartMs >= kReportIntervalMs)
  {
    sendReport();
    intervalStartMs = now;
  }

  if (now - healthStartMs >= 1000)
  {
    Serial.printf("-- %lu polls/s, %lu timed out, %lu send failures\n", (unsigned long)healthPolls,
                  (unsigned long)healthTimeouts, (unsigned long)sendFailures);
    if (healthPolls > 0 && healthTimeouts == healthPolls)
    {
      Serial.println("-- reader not answering: check power, wiring (GPIO18 RX / GPIO17 TX) and baud");
    }
    healthPolls = 0;
    healthTimeouts = 0;
    sendFailures = 0;
    healthStartMs = now;
  }
}
