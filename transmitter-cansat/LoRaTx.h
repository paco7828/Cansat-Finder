class LoRaTx {
private:
  int rxPin;
  int txPin;
  long baudRate;
  int timeOut = 1000;

  void sendCommand(const String &command) {
    Serial2.println(command);
    delay(100);
    while (Serial2.available()) {
      String response = Serial2.readStringUntil('\n');
      response.trim();
      if (response.length() > 0) {
        Serial.println(response);
      }
    }
  }

  String stringToHex(const String &input) {
    String hexResult = "";
    for (int i = 0; i < input.length(); i++) {
      char c = input.charAt(i);
      if ((byte)c < 0x10) {
        hexResult += "0";
      }
      hexResult += String((byte)c, HEX);
    }
    return hexResult;
  }

public:
  LoRaTx(int rx = 16, int tx = 17, long baud = 115200)
    : rxPin(rx), txPin(tx), baudRate(baud) {}

  void begin(const String &frequency = "865375000", const String &bandWidth = "250", const String &sync = "12") {
    Serial2.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    sendCommand("sys reset");
    delay(3000);
    sendCommand("radio set freq " + frequency);
    sendCommand("radio set bw " + bandWidth);
    sendCommand("radio set sync " + sync);
    sendCommand("radio set pa on");
    sendCommand("radio set pwr 20");
    Serial.println("LoRa setup complete.");
  }

  void transmit(const String &message, const int transmitDelay = 0) {
    unsigned long startTime = millis();
    String encodedMessage = stringToHex(message);
    Serial2.println("radio tx " + encodedMessage + " 1");

    // Wait for and process all responses
    bool transmissionComplete = false;
    unsigned long responseTimeout = startTime + this->timeOut;

    while (millis() < responseTimeout && !transmissionComplete) {
      if (Serial2.available()) {
        String response = Serial2.readStringUntil('\n');
        response.trim();
        if (response.length() > 0) {
          Serial.println(response);
          if (response.indexOf("radio_tx_ok") != -1) {
            transmissionComplete = true;
          }
        }
      }
    }

    // Calculate and apply remaining delay
    unsigned long elapsedTime = millis() - startTime;
    if (elapsedTime < transmitDelay) {
      delay(transmitDelay - elapsedTime);
    }
  }
};