#include <BluetoothSerial.h>

class LoRaRx {
private:
  int rxPin;
  int txPin;
  long baudRate;
  String btName;
  BluetoothSerial SerialBT;
  int timeOut = 300;

  void sendCommand(const String &command, bool fromBluetooth = false) {
    Serial2.println(command);

    // Wait for response with timeout
    unsigned long startTime = millis();
    String response = "";

    while (millis() - startTime < this->timeOut) {
      if (Serial2.available()) {
        char c = Serial2.read();
        if (c >= 32 && c <= 126) {
          response += c;
        }
      }

      if (response.endsWith("\n") || response.endsWith("\r")) {
        break;
      }
    }

    response.trim();

    if (response.length() > 0) {
      Serial.println(response);
      if (fromBluetooth) {
        SerialBT.println(response);
      }
    }
    delay(100);
  }

  String hexToString(const String &hexInput) {
    String textResult = "";
    for (int i = 0; i < hexInput.length(); i += 2) {
      String byteHex = hexInput.substring(i, i + 2);
      char c = (char)strtol(byteHex.c_str(), NULL, 16);
      textResult += c;
    }
    return textResult;
  }

  void handleBluetoothCommand(const String &command) {
    String cmdLower = command;
    cmdLower.toLowerCase();

    if (cmdLower.startsWith("radio") || cmdLower.startsWith("sys")) {
      sendCommand(command, true);
    } else {
      SerialBT.println(command);
    }
  }

public:
  LoRaRx(int rx = 16, int tx = 17, long baud = 115200, String btName = "SAS2-SEAT")
    : rxPin(rx), txPin(tx), baudRate(baud), btName(btName) {}

  void begin(const String &frequency = "865375000", const String &bandWidth = "250", const String &sync = "12") {
    Serial2.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    SerialBT.begin(btName);
    Serial.println("Initializing LoRa...");
    sendCommand("sys reset");
    sendCommand("radio set freq " + frequency);
    sendCommand("radio set bw " + bandWidth);
    sendCommand("radio set sync " + sync);
    sendCommand("radio rx 0");
    Serial.println("Setup complete.");
  }

  void listen(bool isRawHex) {
    if (Serial2.available()) {
      String radioData = Serial2.readStringUntil('\r');
      radioData.trim();
      if (radioData.length() > 0) {
        if (isRawHex) {
          Serial.println("Received (HEX): " + radioData);
        } else {
          if (radioData.length() > 9) {
            String formattedData = hexToString(radioData.substring(9));
            Serial.println("Received (TEXT): " + formattedData);
          }
        }
      }
    }

    if (SerialBT.available()) {
      String btCommand = SerialBT.readStringUntil('\n');
      btCommand.trim();
      if (btCommand.length() > 0) {
        handleBluetoothCommand(btCommand);
      }
    }
  }
};