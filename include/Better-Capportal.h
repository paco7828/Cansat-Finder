#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <esp_wifi.h>
#include <functional>
#include <map>

class BetterCapportal
{
private:
  WebServer server;
  DNSServer dnsServer;
  bool apRunning;
  std::map<String, String> formData;
  std::function<void(std::map<String, String> &)> onFormSubmit;
  String customHTML;
  bool hasCustomHTML;

  // AP runtime
  unsigned long runTime = 0;
  unsigned long startTime = 0;

  // Special IP range that works for captive portals
  const IPAddress localIP = IPAddress(4, 3, 2, 1);
  const IPAddress gateway = IPAddress(4, 3, 2, 1);
  const IPAddress subnet = IPAddress(255, 255, 255, 0);
  const String localIPURL = "http://4.3.2.1";

public:
  BetterCapportal() : server(80), apRunning(false), hasCustomHTML(false) {}

  // Start AP with optional password and runtime
  bool begin(String apName = "ESP32-Captive-AP", String apPassword = "", unsigned long runTime = 60000)
  {
    if (apRunning)
      return true;

    this->runTime = runTime;
    this->startTime = millis();

    WiFi.disconnect();
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAPConfig(localIP, gateway, subnet);

    bool success;
    if (apPassword.length() > 0)
      success = WiFi.softAP(apName.c_str(), apPassword.c_str());
    else
      success = WiFi.softAP(apName.c_str());

    if (!success)
      return false;

    // Fix Android issues
    esp_wifi_stop();
    esp_wifi_deinit();
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    config.ampdu_rx_enable = false;
    esp_wifi_init(&config);
    esp_wifi_start();
    delay(100);

    setupCaptivePortal();
    server.begin();
    dnsServer.setTTL(0);
    dnsServer.start(53, "*", localIP);

    apRunning = true;
    return true;
  }

  // Set custom HTML page
  void setHTML(String html)
  {
    customHTML = html;
    hasCustomHTML = true;
  }

  // Handle requests (call in loop)
  void handle()
  {
    if (!apRunning)
      return;

    dnsServer.processNextRequest();
    server.handleClient();

    // Auto-stop AP after runTime
    if (runTime > 0 && millis() - startTime >= runTime)
    {
      stop();
    }
  }

  void onSubmit(std::function<void(std::map<String, String> &)> callback)
  {
    onFormSubmit = callback;
  }

  // Get individual form value
  String getValue(String key)
  {
    return formData[key];
  }

  // Get all form data
  std::map<String, String> &getAllValues()
  {
    return formData;
  }

  // Stop AP
  void stop()
  {
    if (!apRunning)
      return;
    dnsServer.stop();
    server.stop();
    WiFi.softAPdisconnect(true);
    apRunning = false;
  }

  String getIP()
  {
    return localIP.toString();
  }

  bool isRunning()
  {
    return apRunning;
  }

  int clientCount()
  {
    return WiFi.softAPgetStationNum();
  }

private:
  void setupCaptivePortal()
  {
    // Windows 11 captive portal workaround
    server.on("/connecttest.txt", [this]()
              {
      server.sendHeader("Location", "http://logout.net", true);
      server.send(302, "text/plain", ""); });

    // Prevents Windows 10 from repeatedly calling this
    server.on("/wpad.dat", [this]()
              { server.send(404, "text/plain", ""); });

    // Android captive portal detection
    server.on("/generate_204", [this]()
              { handleRoot(); });

    // Microsoft/Windows detection
    server.on("/redirect", [this]()
              { handleRoot(); });

    // Apple/iOS detection
    server.on("/hotspot-detect.html", [this]()
              { handleRoot(); });

    // Firefox detection
    server.on("/canonical.html", [this]()
              { handleRoot(); });

    // Firefox success check
    server.on("/success.txt", [this]()
              { server.send(200, "text/plain", "success"); });

    // Windows NCSI detection
    server.on("/ncsi.txt", [this]()
              { server.send(200, "text/plain", "Microsoft NCSI"); });

    // Return 404 for favicon requests
    server.on("/favicon.ico", [this]()
              { server.send(404, "text/plain", ""); });

    // Main page
    server.on("/", [this]()
              { handleRoot(); });

    server.on("/save", HTTP_POST, [this]()
              { handleSave(); });

    // Catch-all handler redirects to main page
    server.onNotFound([this]()
                      { handleRoot(); });
  }

  void handleRoot()
  {
    String html;

    if (hasCustomHTML)
    {
      html = customHTML;
    }
    else
    {
      // Default WiFi form with better styling
      html = R"(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>WiFi Setup</title><style>*{margin:0;padding:0;box-sizing:border-box}body{font-family:Arial,sans-serif;background:#f5f5f5;padding:20px}.container{max-width:400px;margin:0 auto;background:white;padding:30px;border-radius:8px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}h1{color:#333;margin-bottom:25px;text-align:center}label{display:block;margin:15px 0 5px;font-weight:bold;color:#555}input{width:100%;padding:12px;border:2px solid #ddd;border-radius:4px;font-size:16px}input:focus{outline:none;border-color:#007bff}button{background:#007bff;color:white;padding:12px;border:none;border-radius:4px;font-size:16px;width:100%;margin-top:20px;cursor:pointer}button:hover{background:#0056b3}.error{background:#f8d7da;color:#721c24;padding:12px;border-radius:4px;margin-bottom:20px}</style></head><body><div class='container'><h1>WiFi Configuration</h1><form method='POST' action='/save'><label>WiFi Network Name</label><input type='text' name='ssid' required placeholder='Enter WiFi name'><label>WiFi Password</label><input type='password' name='password' placeholder='Enter WiFi password'><button type='submit'>Connect</button></form></div></body></html>)";
    }

    // Cache control headers to prevent caching
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.send(200, "text/html", html);
  }

  void handleSave()
  {
    // Clear previous data
    formData.clear();

    // Collect all form parameters
    for (int i = 0; i < server.args(); i++)
    {
      formData[server.argName(i)] = server.arg(i);
    }

    // Basic validation for default form
    if (!hasCustomHTML)
    {
      if (!server.hasArg("ssid") || server.arg("ssid").length() == 0)
      {
        String errorHtml = R"(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>WiFi Setup</title><style>*{margin:0;padding:0;box-sizing:border-box}body{font-family:Arial,sans-serif;background:#f5f5f5;padding:20px}.container{max-width:400px;margin:0 auto;background:white;padding:30px;border-radius:8px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}h1{color:#333;margin-bottom:25px;text-align:center}label{display:block;margin:15px 0 5px;font-weight:bold;color:#555}input{width:100%;padding:12px;border:2px solid #ddd;border-radius:4px;font-size:16px}input:focus{outline:none;border-color:#007bff}button{background:#007bff;color:white;padding:12px;border:none;border-radius:4px;font-size:16px;width:100%;margin-top:20px;cursor:pointer}button:hover{background:#0056b3}.error{background:#f8d7da;color:#721c24;padding:12px;border-radius:4px;margin-bottom:20px}</style></head><body><div class='container'><h1>WiFi Configuration</h1><div class='error'>WiFi name is required!</div><form method='POST' action='/save'><label>WiFi Network Name</label><input type='text' name='ssid' required placeholder='Enter WiFi name'><label>WiFi Password</label><input type='password' name='password' placeholder='Enter WiFi password'><button type='submit'>Connect</button></form></div></body></html>)";

        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "-1");
        server.send(400, "text/html", errorHtml);
        return;
      }
    }

    // Call user callback with form data
    if (onFormSubmit)
    {
      onFormSubmit(formData);
    }

    // Show success page
    String successHtml = R"(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Configuration Saved</title><style>*{margin:0;padding:0;box-sizing:border-box}body{font-family:Arial,sans-serif;background:#f5f5f5;padding:20px}.container{max-width:400px;margin:0 auto;background:white;padding:30px;border-radius:8px;box-shadow:0 2px 10px rgba(0,0,0,0.1);text-align:center}.success{background:#d4edda;color:#155724;padding:20px;border-radius:4px;margin-bottom:20px}h1{color:#28a745;margin-bottom:10px}</style></head><body><div class='container'><div class='success'><h1>Settings Saved!</h1><p>Your configuration has been saved successfully.</p></div></div></body></html>)";

    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.send(200, "text/html", successHtml);
  }
};