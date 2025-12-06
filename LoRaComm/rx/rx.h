class LoRaRx {
private:
  int rxPin;
  int txPin;
  long baudRate;

  String hexToString(const String &hexInput) {
    String textResult = "";
    for (int i = 0; i < hexInput.length(); i += 2) {
      String byteHex = hexInput.substring(i, i + 2);
      char c = (char)strtol(byteHex.c_str(), NULL, 16);
      textResult += c;
    }
    return textResult;
  }

  // Clean up the incoming data (remove non-hex characters)
  String cleanHexData(const String &data) {
    String clean = "";
    for (int i = 0; i < data.length(); i++) {
      char c = data.charAt(i);
      if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
        clean += c;
      }
    }
    return clean;
  }

public:
  LoRaRx(int rx = 18, int tx = 17, long baud = 115200)
    : rxPin(rx), txPin(tx), baudRate(baud) {}

  void begin(const String &frequency = "865375000", const String &bandWidth = "250", const String &sync = "12") {
    Serial2.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    delay(1000);

    Serial.println("Initializing WLR089 LoRa Receiver...");

    // Clear any garbage
    while (Serial2.available()) {
      Serial2.read();
    }

    // Send reset
    Serial2.println("sys reset");
    delay(4000);  // WLR089 needs longer reset time

    // Send configuration
    Serial2.println("radio set freq " + frequency);
    delay(100);
    Serial2.println("radio set bw " + bandWidth);
    delay(100);
    Serial2.println("radio set sync " + sync);
    delay(100);

    // Put in CONTINUOUS receive mode (ONCE, at startup!)
    Serial2.println("radio rx 0");
    delay(100);

    Serial.println("LoRa Receiver Ready.");
    Serial.println("Mode: Continuous Receive (radio rx 0)");
  }

  void listen() {
    // Read any available data
    if (Serial2.available()) {
      String incoming = Serial2.readStringUntil('\n');
      incoming.trim();

      // Skip empty lines
      if (incoming.length() == 0) return;

      // Check if this is a data packet
      if (incoming.startsWith("radio_rx")) {
        // Extract hex data (after "radio_rx ")
        String hexData = incoming.substring(9);
        hexData.trim();

        // Clean the hex data
        hexData = cleanHexData(hexData);

        // Convert to text
        String textData = hexToString(hexData);

        // Print the received data
        Serial.println("RECEIVED: " + textData);

        // Parse coordinates if available
        int firstSemi = textData.indexOf(';');
        if (firstSemi != -1) {
          String lat = textData.substring(0, firstSemi);
          String remaining = textData.substring(firstSemi + 1);

          int secondSemi = remaining.indexOf(';');
          if (secondSemi != -1) {
            String lon = remaining.substring(0, secondSemi);
            Serial.println("Parsed: Lat=" + lat + ", Lon=" + lon);
          }
        }

        // DO NOT SEND "radio rx 0" HERE!
        // The module is already in continuous receive mode

      } else if (incoming != "ok" && incoming != "") {
        // Print other messages (but ignore "ok")
        Serial.println("LoRa: " + incoming);
      }
    }
  }
};