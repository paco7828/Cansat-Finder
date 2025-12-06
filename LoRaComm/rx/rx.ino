#include "rx.h"

LoRaRx loraRx(18, 17);

void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for Serial

  Serial.println("\n======================================");
  Serial.println("WLR089 LoRa Receiver");
  Serial.println("======================================");

  loraRx.begin();
}

void loop() {
  loraRx.listen();
  delay(10);  // Small delay
}