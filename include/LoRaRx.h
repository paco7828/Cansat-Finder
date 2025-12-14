#pragma once

class LoRaRx
{
private:
  int rxPin;
  int txPin;
  long baudRate;
  String buffer;
  unsigned long lastReceiveTime = 0;
  bool inContinuousMode = false;

  String hexToString(const String &hexInput)
  {
    String textResult = "";
    for (int i = 0; i < hexInput.length(); i += 2)
    {
      String byteHex = hexInput.substring(i, i + 2);
      char c = (char)strtol(byteHex.c_str(), NULL, 16);
      textResult += c;
    }
    return textResult;
  }

  // Clean hex string from garbage characters
  String cleanHexString(const String &input)
  {
    String result = "";
    for (int i = 0; i < input.length(); i++)
    {
      char c = input.charAt(i);
      if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))
      {
        result += c;
      }
    }
    return result;
  }

  void sendCommand(const String &command)
  {
    Serial2.println(command);
    delay(100);

    // Read response
    unsigned long startTime = millis();
    while (millis() - startTime < 500)
    {
      if (Serial2.available())
      {
        String response = Serial2.readStringUntil('\n');
        response.trim();
        if (response.length() > 0 && response != "ok")
        {
          Serial.println("LoRa CMD: " + response);
        }
      }
    }
  }

public:
  LoRaRx(int rx = 16, int tx = 17, long baud = 115200)
      : rxPin(rx), txPin(tx), baudRate(baud) {}

  void begin(const String &frequency = "865375000", const String &bandWidth = "250", const String &sync = "12")
  {
    Serial2.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    delay(2000);

    Serial.println("Initializing WLR089 LoRa Receiver...");

    // Clear any garbage in buffer
    while (Serial2.available())
    {
      Serial2.read();
    }

    // Factory reset
    Serial2.println("sys factoryRESET");
    delay(4000);

    // Set to LoRa mode
    Serial2.println("radio set mod lora");
    delay(100);
    Serial2.println("radio set freq " + frequency);
    delay(100);
    Serial2.println("radio set bw " + bandWidth);
    delay(100);
    Serial2.println("radio set sf 7");
    delay(100);
    Serial2.println("radio set sync " + sync);
    delay(100);
    Serial2.println("radio set wdt 0");
    delay(100);

    // Put in continuous receive mode
    Serial2.println("radio rx 0");
    delay(100);

    inContinuousMode = true;
    Serial.println("LoRa Receiver Ready (Continuous Mode)");
  }

  String listen()
  {
    String result = "";

    // Read all available data
    while (Serial2.available())
    {
      String line = Serial2.readStringUntil('\n');
      line.trim();

      if (line.length() == 0)
        continue;

      // Check for radio_rx message (data received)
      if (line.startsWith("radio_rx"))
      {
        // Extract hex payload
        String hexData = line.substring(9); // Skip "radio_rx "
        hexData.trim();

        // Clean hex data
        hexData = cleanHexString(hexData);

        // Convert to text
        result = hexToString(hexData);

        if (result.length() > 0)
        {
          Serial.println("LoRa Received: " + result);
          lastReceiveTime = millis();

          // Put back in receive mode if needed
          if (!inContinuousMode)
          {
            delay(50);
            Serial2.println("radio rx 0");
            delay(50);
          }
        }
      }
      // Handle "busy" or other responses
      else if (line == "busy")
      {
        // Module is busy - ignore and try again later
        Serial.println("LoRa: Module busy, retrying...");
        delay(100);
        if (inContinuousMode)
        {
          Serial2.println("radio rx 0");
          delay(50);
        }
      }
      else if (line != "ok" && line.length() > 0)
      {
        // Other responses (print for debugging)
        Serial.println("LoRa: " + line);
      }
    }

    return result;
  }

  // For backward compatibility
  void listen(bool isRawHex)
  {
    if (isRawHex)
    {
      listen();
    }
    else
    {
      String data = listen();
      if (data.length() > 0)
      {
        Serial.println("Received: " + data);
      }
    }
  }
};