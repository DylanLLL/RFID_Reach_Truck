// ESP-NOW pairing between one truck's fork and cabin units.
//
// These are the two bench dev boards (MACs from the esp_get_mac_address env,
// recorded in scribbles.txt). Hardcoding is fine for one pair on a bench; for
// the fleet of 10 it has to become per-unit config -- NVS, or a pairing mode --
// because one firmware image per truck does not scale.

#pragma once

#include <stdint.h>

namespace truck_pairing
{

constexpr uint8_t kForkMac[6] = { 0xE0, 0x72, 0xA1, 0xF4, 0x89, 0x68 };
constexpr uint8_t kCabinMac[6] = { 0xE0, 0x72, 0xA1, 0xCE, 0x8A, 0xEC };

// Both ends pin the radio to this channel. ESP-NOW only hears peers on the
// current channel, and associating to an AP moves it (decision 3) -- so the
// cabin's Firebase sync must return to this channel when it finishes.
constexpr uint8_t kChannel = 1;

}  // namespace truck_pairing
