#pragma once

// HTML for LoRa configuration
constexpr String configHTML = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width,initial-scale=1'>
  <title>CanSat LoRa Configuration</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { 
      font-family: Arial, sans-serif; 
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      padding: 20px; 
      min-height: 100vh;
    }
    .container {
      max-width: 500px;
      margin: 0 auto;
      background: white;
      padding: 30px;
      border-radius: 15px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.2);
    }
    h1 {
      color: #333;
      margin-bottom: 10px;
      text-align: center;
      font-size: 24px;
    }
    .subtitle {
      text-align: center;
      color: #666;
      margin-bottom: 30px;
      font-style: italic;
    }
    label {
      display: block;
      margin: 20px 0 8px;
      font-weight: bold;
      color: #555;
    }
    input, select {
      width: 100%;
      padding: 12px;
      border: 2px solid #ddd;
      border-radius: 8px;
      font-size: 16px;
      transition: border-color 0.3s;
    }
    input:focus, select:focus {
      outline: none;
      border-color: #667eea;
      box-shadow: 0 0 5px rgba(102, 126, 234, 0.3);
    }
    button {
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      color: white;
      padding: 15px;
      border: none;
      border-radius: 8px;
      font-size: 18px;
      font-weight: bold;
      width: 100%;
      margin-top: 25px;
      cursor: pointer;
      transition: transform 0.2s;
    }
    button:hover {
      transform: translateY(-2px);
    }
    .help {
      background: #f8f9fa;
      padding: 15px;
      border-radius: 8px;
      margin-top: 20px;
      font-size: 14px;
      color: #666;
    }
    .current {
      background: #e3f2fd;
      padding: 15px;
      border-radius: 8px;
      margin-bottom: 20px;
    }
    .current h3 {
      color: #1976d2;
      margin-bottom: 10px;
    }
  </style>
</head>
<body>
  <div class='container'>
    <h1>🛰️ CanSat Finder</h1>
    <div class='subtitle'>LoRa Configuration Panel</div>
    
    <div class='current'>
      <h3>Current Settings:</h3>
      <div>Frequency: <strong>)"
                      + loraConfig.frequency + R"(</strong> Hz</div>
      <div>Bandwidth: <strong>)"
                      + loraConfig.bandwidth + R"(</strong> kHz</div>
      <div>Sync Word: <strong>)"
                      + loraConfig.sync + R"(</strong></div>
      <div>Baudrate: <strong>)"
                      + String(loraConfig.baudrate) + R"(</strong> bps</div>
    </div>

    <form method='POST' action='/save'>
      <label>📡 Frequency (Hz)</label>
      <input type='number' name='frequency' value=')"
                      + loraConfig.frequency + R"(' required 
             min='862000000' max='1020000000' placeholder='e.g., 865375000'>
      
      <label>📊 Bandwidth (kHz)</label>
      <select name='bandwidth' required>
        <option value='125' )"
                      + (loraConfig.bandwidth == "125" ? "selected" : "") + R"(>125 kHz</option>
        <option value='250' )"
                      + (loraConfig.bandwidth == "250" ? "selected" : "") + R"(>250 kHz</option>
        <option value='500' )"
                      + (loraConfig.bandwidth == "500" ? "selected" : "") + R"(>500 kHz</option>
      </select>
      
      <label>🔗 Sync Word (Hex)</label>
      <input type='text' name='sync' value=')"
                      + loraConfig.sync + R"(' required 
             pattern='[0-9A-Fa-f]+' placeholder='e.g., 12'>
      
      <label>⚡ Baudrate (bps)</label>
      <select name='baudrate' required>
        <option value='9600' )"
                      + (loraConfig.baudrate == 9600 ? "selected" : "") + R"(>9600 bps</option>
        <option value='19200' )"
                      + (loraConfig.baudrate == 19200 ? "selected" : "") + R"(>19200 bps</option>
        <option value='38400' )"
                      + (loraConfig.baudrate == 38400 ? "selected" : "") + R"(>38400 bps</option>
        <option value='57600' )"
                      + (loraConfig.baudrate == 57600 ? "selected" : "") + R"(>57600 bps</option>
        <option value='115200' )"
                      + (loraConfig.baudrate == 115200 ? "selected" : "") + R"(>115200 bps</option>
      </select>
      
      <button type='submit'>💾 Save Configuration</button>
    </form>
    
    <div class='help'>
      <strong>💡 Tips:</strong><br>
      • Frequency must match your LoRa module's band<br>
      • Higher bandwidth = faster data, shorter range<br>
      • Sync word must match transmitter<br>
      • Higher baudrate = faster serial communication
    </div>
  </div>
</body>
</html>
)";