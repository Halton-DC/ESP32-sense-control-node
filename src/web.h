/**
 * web.h - HTTPS server + authenticated REST API.
 *
 * Serves the settings UI and a JSON REST API over TLS (esp_https_server).
 * Authentication is a session cookie issued by POST /api/login; the cookie is
 * HttpOnly + Secure + SameSite=Strict. All /api/* endpoints except /api/login
 * require a valid session.
 */
#pragma once
#include <Arduino.h>

// Live runtime snapshot the web layer renders. Filled by appGetStatus().
struct WebStatus {
  float tempC, tempF, humidity;
  int   airflow;
  bool  contact1, contact2, relay1, relay2, link;
  char  ip[16];         // primary/active IP (for the header)
  char  uptime[24];
  // Network detail
  char  mode[12];       // "Ethernet" | "WiFi" | "Setup AP" | "Offline"
  bool  ethLink;
  char  ethIp[16];
  bool  wifiUp;
  char  wifiIp[16];
  int   wifiRssi;
  bool  apActive;
  char  ssid[33];       // connected/AP SSID
  char  mac[18];        // primary MAC address
};

// Start the HTTPS server (call after network is up and certsBegin() succeeded).
void webBegin();
bool webStarted();

// Push a telemetry frame to all connected SSE clients. Call periodically from
// the main loop (e.g. ~1 Hz). No-op if there are no stream subscribers.
void webPushTelemetry();

// --- Implemented in main.cpp (runtime data + actuation) ---
void appGetStatus(WebStatus *s);
void appSetRelay(int idx, bool on); // idx 1..2; applies to GPIO and persists
void appReboot();
