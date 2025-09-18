#include "rx.h"

LoRaRx loraRx;

void setup()
{
  Serial.begin(115200);
  loraRx.begin();
}

void loop()
{
  loraRx.listen(false);
}