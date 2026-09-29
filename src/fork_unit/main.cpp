// Fork unit -- placeholder.
//
// This is the battery-powered sensor on the fork carriage. Per decision 2 in
// CLAUDE.md it stays deliberately dumb: it never associates to WiFi, holds no
// lookup table, and does no arbitration. Its whole job is
//
//   ultrasonic detects a beam
//     -> open a 150-300 ms read window on the R200
//     -> collect the tag inventory
//     -> send {seq, [(epc, rssi, hits)], gate_state} over ESP-NOW
//
// and nothing else, because it is the hardest unit in the system to reflash.
// All judgement -- RSSI arbitration, table lookup, QR generation, staleness --
// lives on the cabin unit, which can be reached without unbolting anything.
//
// Nothing is implemented yet. The R200 driver is proven separately in
// src/R200.cpp (env `R200`); the ESP-NOW link is proven in src/ESP-NOW_test.cpp.
// This file is where those two get joined.

#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  delay(2000);  // let the host attach before the first print

  Serial.println();
  Serial.println("=== fork unit: placeholder, no functionality yet ===");
  Serial.printf("psram: %lu bytes (%s)\n", (unsigned long)ESP.getPsramSize(),
                psramFound() ? "detected" : "NOT FOUND");
}

void loop()
{
  delay(1000);
}
