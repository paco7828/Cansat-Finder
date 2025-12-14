#pragma once

#include "config.h"

// Function to generate HTML for LoRa configuration
String generateConfigHTML(const LoRaConfig &config)
{
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>CanSat LoRa Configuration</title>";
  html += "<style>";
  html += "*{margin:0;padding:0;box-sizing:border-box}";
  html += "body{font-family:Arial,sans-serif;background:#f5f5f5;padding:20px;min-height:100vh}";
  html += ".container{max-width:500px;margin:0 auto;background:white;padding:30px;border-radius:15px;box-shadow:0 10px 30px rgba(0,0,0,0.2)}";
  html += "h1{color:#333;margin-bottom:10px;text-align:center;font-size:24px}";
  html += ".subtitle{text-align:center;color:#666;margin-bottom:30px;font-style:italic}";
  html += "label{display:block;margin:20px 0 8px;font-weight:bold;color:#555}";
  html += "input,select{width:100%;padding:12px;border:2px solid #ddd;border-radius:8px;font-size:16px}";
  html += "button{background:#007bff;color:white;padding:15px;border:none;border-radius:8px;font-size:18px;width:100%;margin-top:25px;cursor:pointer}";
  html += "button:hover{background:#0056b3}";
  html += ".current{background:#e3f2fd;padding:15px;border-radius:8px;margin-bottom:20px}";
  html += ".current h3{color:#1976d2;margin-bottom:10px}";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>🛰️ CanSat Finder</h1>";
  html += "<div class='subtitle'>LoRa Configuration Panel</div>";

  html += "<div class='current'>";
  html += "<h3>Current Settings:</h3>";
  html += "<div>Frequency: <strong>";
  html += config.frequency;
  html += "</strong> Hz</div>";
  html += "<div>Bandwidth: <strong>";
  html += config.bandwidth;
  html += "</strong> kHz</div>";
  html += "<div>Sync Word: <strong>";
  html += config.sync;
  html += "</strong></div>";
  html += "<div>Baudrate: <strong>";
  html += String(config.baudrate);
  html += "</strong> bps</div>";
  html += "</div>";

  html += "<form method='POST' action='/save'>";
  html += "<label>📡 Frequency (Hz)</label>";
  html += "<input type='number' name='frequency' value='";
  html += config.frequency;
  html += "' required min='862000000' max='1020000000' placeholder='e.g., 865375000'>";

  html += "<label>📊 Bandwidth (kHz)</label>";
  html += "<select name='bandwidth' required>";
  html += "<option value='125'";
  if (config.bandwidth == "125")
    html += " selected";
  html += ">125 kHz</option>";
  html += "<option value='250'";
  if (config.bandwidth == "250")
    html += " selected";
  html += ">250 kHz</option>";
  html += "<option value='500'";
  if (config.bandwidth == "500")
    html += " selected";
  html += ">500 kHz</option>";
  html += "</select>";

  html += "<label>🔗 Sync Word (Hex)</label>";
  html += "<input type='text' name='sync' value='";
  html += config.sync;
  html += "' required pattern='[0-9A-Fa-f]+' placeholder='e.g., 12'>";

  html += "<label>⚡ Baudrate (bps)</label>";
  html += "<select name='baudrate' required>";
  html += "<option value='9600'";
  if (config.baudrate == 9600)
    html += " selected";
  html += ">9600 bps</option>";
  html += "<option value='19200'";
  if (config.baudrate == 19200)
    html += " selected";
  html += ">19200 bps</option>";
  html += "<option value='38400'";
  if (config.baudrate == 38400)
    html += " selected";
  html += ">38400 bps</option>";
  html += "<option value='57600'";
  if (config.baudrate == 57600)
    html += " selected";
  html += ">57600 bps</option>";
  html += "<option value='115200'";
  if (config.baudrate == 115200)
    html += " selected";
  html += ">115200 bps</option>";
  html += "</select>";

  html += "<button type='submit'>💾 Save Configuration</button>";
  html += "</form>";

  html += "<div style='background:#f8f9fa;padding:15px;border-radius:8px;margin-top:20px;font-size:14px;color:#666'>";
  html += "<strong>💡 Tips:</strong><br>";
  html += "• Frequency must match your LoRa module's band<br>";
  html += "• Higher bandwidth = faster data, shorter range<br>";
  html += "• Sync word must match transmitter<br>";
  html += "• Higher baudrate = faster serial communication";
  html += "</div>";
  html += "</div></body></html>";

  return html;
}