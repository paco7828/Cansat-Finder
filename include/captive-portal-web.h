#pragma once

#include "config.h"
#include <Arduino.h>

String generateConfigHTML(const LoRaConfig &config)
{
  const char *htmlTemplate = R"(<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>CanSat LoRa Configuration</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;background:#f5f5f5;padding:20px;min-height:100vh}
.container{max-width:500px;margin:0 auto;background:white;padding:30px;border-radius:15px;box-shadow:0 10px 30px rgba(0,0,0,0.2)}
h1{color:#333;margin-bottom:10px;text-align:center;font-size:24px}
.subtitle{text-align:center;color:#666;margin-bottom:30px;font-style:italic}
label{display:block;margin:20px 0 8px;font-weight:bold;color:#555}
input,select{width:100%;padding:12px;border:2px solid #ddd;border-radius:8px;font-size:16px}
button{background:#007bff;color:white;padding:15px;border:none;border-radius:8px;font-size:18px;width:100%;margin-top:25px;cursor:pointer}
button:hover{background:#0056b3}
</style></head><body>
<div class='container'>
<h1>🛰️ CanSat Finder</h1>
<div class='subtitle'>LoRa Configuration Panel</div>
<form method='POST' action='/save'>
<label>📡 Frequency (Hz)</label>
<input type='number' name='frequency' value='%s' required min='862000000' max='1020000000' placeholder='e.g., 865375000'>
<label>📊 Bandwidth (kHz)</label>
<select name='bandwidth' required>
<option value='125'%s>125 kHz</option>
<option value='250'%s>250 kHz</option>
<option value='500'%s>500 kHz</option>
</select>
<label>🔗 Sync Word (Hex)</label>
<input type='text' name='sync' value='%s' required pattern='[0-9A-Fa-f]+' placeholder='e.g., 12'>
<label>⚡ Baudrate (bps)</label>
<select name='baudrate' required>
<option value='9600'%s>9600 bps</option>
<option value='19200'%s>19200 bps</option>
<option value='38400'%s>38400 bps</option>
<option value='57600'%s>57600 bps</option>
<option value='115200'%s>115200 bps</option>
</select>
<button type='submit'>💾 Save Configuration</button>
</form>
</div></body></html>)";

  char buffer[4096];
  snprintf(buffer, sizeof(buffer), htmlTemplate,
           config.frequency.c_str(),
           config.bandwidth == "125" ? " selected" : "",
           config.bandwidth == "250" ? " selected" : "",
           config.bandwidth == "500" ? " selected" : "",
           config.sync.c_str(),
           config.baudrate == 9600 ? " selected" : "",
           config.baudrate == 19200 ? " selected" : "",
           config.baudrate == 38400 ? " selected" : "",
           config.baudrate == 57600 ? " selected" : "",
           config.baudrate == 115200 ? " selected" : "");

  return String(buffer);
}