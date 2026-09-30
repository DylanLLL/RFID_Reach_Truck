// Fork -> cabin read report, sent over ESP-NOW.
//
// Shared by both units so the two ends cannot drift apart. Any change to the
// layout must bump kProtocolVersion: the cabin drops versions it doesn't know
// rather than misreading them.
//
// A report is raw evidence -- per-EPC hit count and mean RSSI over a short
// interval, nothing decided. How much evidence to weigh and what counts as a
// confident read is policy, and lives on the cabin unit (decision 2).

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace truck_link
{

constexpr uint8_t kProtocolVersion = 1;

// 96-bit EPC. The R200 driver rejects tags declaring any other length.
constexpr size_t kEpcLength = 12;

// Distinct EPCs per report. Two per beam are expected; the rest is headroom
// for neighbouring bays and for UHF tags on the goods themselves.
constexpr size_t kMaxTags = 8;

struct __attribute__((packed)) TagReport
{
  uint8_t epc[kEpcLength];
  int8_t rssi;   // mean over this report's hits, dBm
  uint8_t hits;  // times this EPC was read during the report interval
};

struct __attribute__((packed)) ReadReport
{
  uint8_t version;
  uint8_t tagCount;  // valid entries in tags[]; only these go over the air
  uint8_t polls;     // inventory rounds run, so hits can be read as a rate
  uint8_t dropped;   // reads discarded because tags[] was already full
  uint32_t bootId;   // random per fork boot, so the cabin can tell a reset from a replay
  uint32_t seq;      // +1 per report, restarting at 1 on every boot
  TagReport tags[kMaxTags];
};

constexpr size_t kHeaderSize = offsetof(ReadReport, tags);

constexpr size_t reportSize(uint8_t tagCount)
{
  return kHeaderSize + tagCount * sizeof(TagReport);
}

static_assert(sizeof(ReadReport) <= 250, "exceeds the ESP-NOW payload limit");

// "E28069150000402092916188". Uppercase, so it also encodes as a QR in
// alphanumeric mode.
inline void epcToHex(const uint8_t* epc, char (&out)[kEpcLength * 2 + 1])
{
  static const char kDigits[] = "0123456789ABCDEF";
  for (size_t i = 0; i < kEpcLength; i++)
  {
    out[2 * i] = kDigits[epc[i] >> 4];
    out[2 * i + 1] = kDigits[epc[i] & 0x0F];
  }
  out[kEpcLength * 2] = '\0';
}

}  // namespace truck_link
