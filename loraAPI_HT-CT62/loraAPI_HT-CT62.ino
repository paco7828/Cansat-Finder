/*
  loraAPI – LoRa Wireless Shell for HT-CT62
  ==========================================
  Hardware : Heltec HT-CT62 (ESP32-C3 + SX1262)
  Board    : ESP32C3 Dev Module  |  USB CDC ON Boot: Enabled

  Shared library: heltecAPI  (../heltecAPI/src/)
  Install heltecAPI in Arduino/libraries/ OR use relative path below.

  Board resource URL:
    https://resource.heltec.cn/download/package_heltec_esp32_index.json
*/

#include <RadioLib.h>
#include <SPI.h>

// ---- heltecAPI shared library (installed in Arduino/libraries/) ----------
#include <HeltecAPI.h>
#include <HeltecCmdInterpreter.h>

#define LORA_API_VERSION "v2.0.0"  // major: migrated to shared heltecAPI lib

// ---- HT-CT62 pin definitions --------------------------------------------
#define PIN_SCK 10
#define PIN_MISO 6
#define PIN_MOSI 7
#define PIN_NSS 8
#define PIN_DIO1 3
#define PIN_RST 5
#define PIN_BUSY 4

// ---- Hardware UART (for external MCU) -----------------------------------
#define UART_RX 20  // GPIO20
#define UART_TX 21  // GPIO21

// ---- Radio & wrapper ----------------------------------------------------
SX1262 radioMod = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
HeltecWrapper heltecLora(&radioMod);

// ---- Default LoRa configuration -----------------------------------------
// freq   bw     sf  cr  sync   pwr  preamble  tcxo  crc
LoRaConfig loraCfg = { 868.0, 125.0, 7, 5, 0x12, 14, 8, 1.6, 2 };

// =========================================================================
void setup() {
  Serial.begin(115200);                                 // USB CDC
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);  // Hardware UART

  // Wait up to 3 s for at least one port to come up
  while (!Serial && !Serial1 && millis() < 3000) delay(10);

  serialPrintln("loraAPI " + String(LORA_API_VERSION));
  delay(200);

  // SPI init (must be done manually on HT-CT62 – no heltec_unofficial here)
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  // Hardware reset of SX1262
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(50);
  digitalWrite(PIN_RST, HIGH);
  delay(100);

  heltecLora.setLogging(true);
  heltecLora.setTXTimeout(5000);

  if (heltecLora.begin(loraCfg) != RADIOLIB_ERR_NONE) {
    while (true) {
      serialPrintln("CRITICAL_ERROR: Radio hardware failure!");
      delay(2000);
    }
  }

  // Transzparens CMD: bridge a MAIN MCU felé.
  // Minden LoRa-n érkező CMD: csomag változtatás nélkül továbbítódik Serial1-en.
  // A CAN/szenzor parancsok értelmezése a MAIN MCU CommandInterpreter-ében történik.
  // A "CMD:" prefix visszakerül, hogy a pcb-test CommandInterpreter felismerje.
  onLoRaRxCmd = [](HeltecWrapper& /*lora*/, const String& cmd) {
    Serial1.println("CMD:" + cmd);
  };
}

void loop() {
  heltecLora.update();                             // state machine & timers
  HeltecCmdInterpreter::handleSerial(heltecLora);  // command parsing
}