#include "web.h"
#include "webui.h"
#include "settings.h"
#include "certs.h"
#include "integrations.h"
#include <esp_https_server.h>
#include <esp_random.h>
#include <ArduinoJson.h>
#include <IPAddress.h>

static httpd_handle_t s_server = nullptr;
static bool s_started = false;

// --- Sessions -------------------------------------------------------------
#define SESSION_SLOTS  4
#define SESSION_TTL_MS (30 * 60 * 1000UL)
struct Session { char tok[33]; uint32_t expires; bool used; };
static Session s_sessions[SESSION_SLOTS];

static void makeToken(char *out33) {
  uint8_t b[16];
  esp_fill_random(b, sizeof(b));
  static const char *hex = "0123456789abcdef";
  for (int i = 0; i < 16; i++) { out33[i * 2] = hex[b[i] >> 4]; out33[i * 2 + 1] = hex[b[i] & 0xF]; }
  out33[32] = 0;
}

static const char *createSession() {
  int slot = -1;
  uint32_t now = millis();
  for (int i = 0; i < SESSION_SLOTS; i++) {
    if (!s_sessions[i].used || (int32_t)(now - s_sessions[i].expires) > 0) { slot = i; break; }
  }
  if (slot < 0) {
    // All slots active: evict the one expiring soonest.
    slot = 0;
    for (int i = 1; i < SESSION_SLOTS; i++)
      if ((int32_t)(s_sessions[i].expires - s_sessions[slot].expires) < 0) slot = i;
  }
  makeToken(s_sessions[slot].tok);
  s_sessions[slot].expires = now + SESSION_TTL_MS;
  s_sessions[slot].used = true;
  return s_sessions[slot].tok;
}

static bool validateToken(const char *tok) {
  if (!tok || strlen(tok) != 32) return false;
  uint32_t now = millis();
  for (int i = 0; i < SESSION_SLOTS; i++) {
    if (s_sessions[i].used && (int32_t)(now - s_sessions[i].expires) <= 0 &&
        strncmp(s_sessions[i].tok, tok, 32) == 0) {
      s_sessions[i].expires = now + SESSION_TTL_MS; // sliding renewal
      return true;
    }
  }
  return false;
}

static void clearSession(const char *tok) {
  for (int i = 0; i < SESSION_SLOTS; i++)
    if (s_sessions[i].used && strncmp(s_sessions[i].tok, tok, 32) == 0) s_sessions[i].used = false;
}

// --- Helpers --------------------------------------------------------------
static bool getCookieToken(httpd_req_t *req, char *out, size_t len) {
  return httpd_req_get_cookie_val(req, "hdcsession", out, &len) == ESP_OK;
}

static bool authed(httpd_req_t *req) {
  char tok[64];
  if (!getCookieToken(req, tok, sizeof(tok))) return false;
  return validateToken(tok);
}

static esp_err_t denyAuth(httpd_req_t *req) {
  httpd_resp_set_status(req, "401 Unauthorized");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"error\":\"unauthorized\"}");
  return ESP_OK;
}

static bool readBody(httpd_req_t *req, char *buf, size_t max) {
  int total = req->content_len;
  if (total <= 0 || total >= (int)max) return false;
  int off = 0;
  while (off < total) {
    int r = httpd_req_recv(req, buf + off, total - off);
    if (r <= 0) return false;
    off += r;
  }
  buf[total] = 0;
  return true;
}

static esp_err_t sendJson(httpd_req_t *req, JsonDocument &doc) {
  String out;
  serializeJson(doc, out);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, out.c_str());
}

static esp_err_t sendErr(httpd_req_t *req, const char *status, const char *msg) {
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "application/json");
  String s = String("{\"error\":\"") + msg + "\"}";
  httpd_resp_sendstr(req, s.c_str());
  return ESP_OK;
}

static String ipToStr(uint32_t v) { return IPAddress(v).toString(); }
static uint32_t strToIp(const char *s) { IPAddress a; a.fromString(s); return (uint32_t)a; }

// --- Handlers -------------------------------------------------------------
static esp_err_t hRoot(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, WEBUI_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t hLogin(httpd_req_t *req) {
  char body[256];
  if (!readBody(req, body, sizeof(body))) return sendErr(req, "400 Bad Request", "bad body");
  JsonDocument in;
  if (deserializeJson(in, body)) return sendErr(req, "400 Bad Request", "bad json");
  const char *u = in["username"] | "";
  const char *p = in["password"] | "";
  if (strcmp(u, g_settings.adminUser) != 0 || !settingsVerifyPassword(p))
    return sendErr(req, "401 Unauthorized", "invalid credentials");

  const char *tok = createSession();
  char cookie[128];
  snprintf(cookie, sizeof(cookie),
           "hdcsession=%s; Path=/; Max-Age=1800; HttpOnly; Secure; SameSite=Strict", tok);
  httpd_resp_set_hdr(req, "Set-Cookie", cookie);
  JsonDocument out; out["ok"] = true; out["mustChangePassword"] = g_settings.mustChangePassword;
  return sendJson(req, out);
}

static esp_err_t hLogout(httpd_req_t *req) {
  char tok[64];
  if (getCookieToken(req, tok, sizeof(tok))) clearSession(tok);
  httpd_resp_set_hdr(req, "Set-Cookie", "hdcsession=; Path=/; Max-Age=0; HttpOnly; Secure; SameSite=Strict");
  httpd_resp_sendstr(req, "{\"ok\":true}");
  return ESP_OK;
}

static void buildStatusDoc(JsonDocument &d) {
  WebStatus st; appGetStatus(&st);
  d["tempC"] = st.tempC; d["tempF"] = st.tempF; d["humidity"] = st.humidity;
  d["airflow"] = st.airflow; d["contact1"] = st.contact1; d["contact2"] = st.contact2;
  d["relay1"] = st.relay1; d["relay2"] = st.relay2; d["link"] = st.link;
  d["ip"] = st.ip; d["uptime"] = st.uptime;
  d["mode"] = st.mode;
  d["ethLink"] = st.ethLink; d["ethIp"] = st.ethIp;
  d["wifiUp"] = st.wifiUp; d["wifiIp"] = st.wifiIp; d["wifiRssi"] = st.wifiRssi;
  d["apActive"] = st.apActive; d["ssid"] = st.ssid;
  d["mustChangePassword"] = g_settings.mustChangePassword;
  d["mac"] = st.mac;
}

static esp_err_t hStatus(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  JsonDocument d; buildStatusDoc(d);
  return sendJson(req, d);
}

// --- Server-Sent Events telemetry stream ---
#define SSE_MAX 2
static httpd_req_t   *s_sse[SSE_MAX] = {nullptr};
static SemaphoreHandle_t s_sseLock = nullptr;

static esp_err_t hStream(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  // Set headers (flushed on the first chunk sent from the loop task), then
  // detach the request so the httpd worker is freed. Do NOT send from here —
  // returning after a send would let the framework finalize the response.
  httpd_resp_set_type(req, "text/event-stream");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  httpd_req_t *copy = nullptr;
  if (httpd_req_async_handler_begin(req, &copy) != ESP_OK) return ESP_FAIL;

  if (!s_sseLock) s_sseLock = xSemaphoreCreateMutex();
  bool placed = false;
  xSemaphoreTake(s_sseLock, portMAX_DELAY);
  for (int i = 0; i < SSE_MAX; i++) {
    if (!s_sse[i]) { s_sse[i] = copy; placed = true; break; }
  }
  xSemaphoreGive(s_sseLock);
  if (!placed) { httpd_req_async_handler_complete(copy); return ESP_OK; } // at capacity → client polls
  Serial.println("[SSE] client subscribed");
  return ESP_OK;
}

void webPushTelemetry() {
  if (!s_sseLock) return;
  bool any = false;
  for (int i = 0; i < SSE_MAX; i++) if (s_sse[i]) { any = true; break; }
  if (!any) return;

  JsonDocument d; buildStatusDoc(d);
  String js; serializeJson(d, js);            // serializeJson overwrites, so build separately
  String frame; frame.reserve(js.length() + 12);
  frame = "data: "; frame += js; frame += "\n\n";

  xSemaphoreTake(s_sseLock, portMAX_DELAY);
  for (int i = 0; i < SSE_MAX; i++) {
    if (!s_sse[i]) continue;
    esp_err_t e = httpd_resp_send_chunk(s_sse[i], frame.c_str(), frame.length());
    if (e != ESP_OK) {
      Serial.printf("[SSE] send failed (%d) - dropping client\n", e);
      httpd_req_async_handler_complete(s_sse[i]); // client gone → release
      s_sse[i] = nullptr;
    }
  }
  xSemaphoreGive(s_sseLock);
}

static esp_err_t hGetSettings(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  JsonDocument d;
  d["nodeName"] = g_settings.nodeName;
  d["location"] = g_settings.location;
  d["contact"]  = g_settings.contact;
  d["hostname"] = g_settings.hostname;
  d["firmware"] = SOFTWARE_VERSION_STR;
  d["build"] = BUILD_NUMBER;
  d["dhcpEnabled"] = g_settings.dhcpEnabled;
  d["staticIp"]   = ipToStr(g_settings.staticIp);
  d["staticMask"] = ipToStr(g_settings.staticMask);
  d["staticGw"]   = ipToStr(g_settings.staticGw);
  d["staticDns"]  = ipToStr(g_settings.staticDns);
  d["wifiEnabled"] = g_settings.wifiEnabled;
  d["wifiSsid"]    = g_settings.wifiSsid;
  d["wifiConfigured"] = strlen(g_settings.wifiPass) > 0;
  d["snmpEnabled"] = g_settings.snmpEnabled;
  d["snmpWriteEnabled"] = g_settings.snmpWriteEnabled;
  d["snmpReadCommunity"]  = g_settings.snmpReadCommunity;
  d["snmpWriteCommunity"] = g_settings.snmpWriteCommunity;
  d["relay1Name"] = g_settings.relay1Name;
  d["relay2Name"] = g_settings.relay2Name;
  d["adminUser"]  = g_settings.adminUser;
  d["sensorIntervalMs"] = g_settings.sensorIntervalMs;
  d["tempUnitF"] = g_settings.tempUnitF;
  d["tempOffset"] = g_settings.tempOffC10 / 10.0;
  d["humOffset"] = g_settings.humOff10 / 10.0;
  d["airflowOffset"] = g_settings.airflowOff / 100.0;
  d["metricsEnabled"] = g_settings.metricsEnabled;
  d["metricsToken"] = g_settings.metricsToken;
  d["mustChangePassword"] = g_settings.mustChangePassword;
  return sendJson(req, d);
}

static void copyStr(char *dst, const char *src, size_t n) {
  if (!src) return; strncpy(dst, src, n - 1); dst[n - 1] = 0;
}

static esp_err_t hSetSettings(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  char body[1024];
  if (!readBody(req, body, sizeof(body))) return sendErr(req, "400 Bad Request", "bad body");
  JsonDocument in;
  if (deserializeJson(in, body)) return sendErr(req, "400 Bad Request", "bad json");

  settingsLock();
  if (in["nodeName"].is<const char *>()) copyStr(g_settings.nodeName, in["nodeName"], sizeof(g_settings.nodeName));
  if (in["hostname"].is<const char *>()) settingsSanitizeHostname(in["hostname"], g_settings.hostname, sizeof(g_settings.hostname));
  if (in["location"].is<const char *>()) copyStr(g_settings.location, in["location"], sizeof(g_settings.location));
  if (in["contact"].is<const char *>())  copyStr(g_settings.contact,  in["contact"],  sizeof(g_settings.contact));
  if (in["sensorIntervalMs"].is<int>())  g_settings.sensorIntervalMs = constrain((int)in["sensorIntervalMs"], 250, 60000);
  if (in["tempUnitF"].is<bool>())        g_settings.tempUnitF = in["tempUnitF"];
  if (!in["tempOffset"].isNull())    g_settings.tempOffC10 = constrain((int)lroundf((float)in["tempOffset"] * 10), -200, 200);
  if (!in["humOffset"].isNull())     g_settings.humOff10   = constrain((int)lroundf((float)in["humOffset"] * 10), -200, 200);
  if (!in["airflowOffset"].isNull()) g_settings.airflowOff = constrain((int)lroundf((float)in["airflowOffset"] * 100), -1000, 1000);
  if (in["metricsEnabled"].is<bool>())   g_settings.metricsEnabled = in["metricsEnabled"];
  if (in["metricsToken"].is<const char *>()) copyStr(g_settings.metricsToken, in["metricsToken"], sizeof(g_settings.metricsToken));

  if (in["dhcpEnabled"].is<bool>()) g_settings.dhcpEnabled = in["dhcpEnabled"];
  if (in["staticIp"].is<const char *>())   g_settings.staticIp   = strToIp(in["staticIp"]);
  if (in["staticMask"].is<const char *>()) g_settings.staticMask = strToIp(in["staticMask"]);
  if (in["staticGw"].is<const char *>())   g_settings.staticGw   = strToIp(in["staticGw"]);
  if (in["staticDns"].is<const char *>())  g_settings.staticDns  = strToIp(in["staticDns"]);

  if (in["wifiEnabled"].is<bool>()) g_settings.wifiEnabled = in["wifiEnabled"];
  if (in["wifiSsid"].is<const char *>()) copyStr(g_settings.wifiSsid, in["wifiSsid"], sizeof(g_settings.wifiSsid));
  // Only overwrite the stored WiFi password when a non-empty one is provided.
  if (in["wifiPass"].is<const char *>()) {
    const char *wp = in["wifiPass"];
    if (wp && wp[0]) copyStr(g_settings.wifiPass, wp, sizeof(g_settings.wifiPass));
  }

  if (in["snmpEnabled"].is<bool>())      g_settings.snmpEnabled = in["snmpEnabled"];
  if (in["snmpWriteEnabled"].is<bool>()) g_settings.snmpWriteEnabled = in["snmpWriteEnabled"];
  if (in["snmpReadCommunity"].is<const char *>())  copyStr(g_settings.snmpReadCommunity,  in["snmpReadCommunity"],  sizeof(g_settings.snmpReadCommunity));
  if (in["snmpWriteCommunity"].is<const char *>()) copyStr(g_settings.snmpWriteCommunity, in["snmpWriteCommunity"], sizeof(g_settings.snmpWriteCommunity));

  if (in["relay1Name"].is<const char *>()) copyStr(g_settings.relay1Name, in["relay1Name"], sizeof(g_settings.relay1Name));
  if (in["relay2Name"].is<const char *>()) copyStr(g_settings.relay2Name, in["relay2Name"], sizeof(g_settings.relay2Name));

  bool ok = settingsSave();
  settingsUnlock();
  JsonDocument out; out["ok"] = ok; out["rebootRequired"] = true;
  return sendJson(req, out);
}

static esp_err_t hRelay(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  char body[128];
  if (!readBody(req, body, sizeof(body))) return sendErr(req, "400 Bad Request", "bad body");
  JsonDocument in;
  if (deserializeJson(in, body)) return sendErr(req, "400 Bad Request", "bad json");
  int idx = in["relay"] | 0;
  bool state = (int)(in["state"] | 0) != 0;
  if (idx != 1 && idx != 2) return sendErr(req, "400 Bad Request", "relay must be 1 or 2");
  appSetRelay(idx, state);
  JsonDocument out; out["ok"] = true; out["relay"] = idx; out["state"] = state;
  return sendJson(req, out);
}

static esp_err_t hPassword(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  char body[256];
  if (!readBody(req, body, sizeof(body))) return sendErr(req, "400 Bad Request", "bad body");
  JsonDocument in;
  if (deserializeJson(in, body)) return sendErr(req, "400 Bad Request", "bad json");
  const char *cur = in["current"] | "";
  const char *nw  = in["new"] | "";
  const char *user = in["username"] | g_settings.adminUser;
  if (!settingsVerifyPassword(cur)) return sendErr(req, "403 Forbidden", "current password incorrect");
  if (strlen(nw) < 8)  return sendErr(req, "400 Bad Request", "password too short (min 8)");
  if (strlen(nw) > 63) return sendErr(req, "400 Bad Request", "password too long (max 63)");
  if (strlen(user) < 1 || strlen(user) > 31) return sendErr(req, "400 Bad Request", "username must be 1-31 chars");
  settingsLock();
  copyStr(g_settings.adminUser, user, sizeof(g_settings.adminUser));
  settingsSetPassword(nw);
  g_settings.mustChangePassword = false;
  settingsSave();
  settingsUnlock();
  httpd_resp_sendstr(req, "{\"ok\":true}");
  return ESP_OK;
}

static esp_err_t hReboot(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  httpd_resp_sendstr(req, "{\"ok\":true}");
  delay(300);
  appReboot();
  return ESP_OK;
}

// --- Prometheus /metrics ---
static String promEscape(const char *s) {
  String o;
  for (const char *p = s; *p; p++) {
    if (*p == '\\' || *p == '"') { o += '\\'; o += *p; }
    else if (*p == '\n') o += "\\n";
    else o += *p;
  }
  return o;
}

static bool metricsAuthed(httpd_req_t *req) {
  if (strlen(g_settings.metricsToken) == 0) return true; // open (trusted VLAN)
  char h[96];
  if (httpd_req_get_hdr_value_str(req, "Authorization", h, sizeof(h)) == ESP_OK) {
    if (strncmp(h, "Bearer ", 7) == 0 && strcmp(h + 7, g_settings.metricsToken) == 0) return true;
  }
  return authed(req); // also allow a logged-in browser session
}

static esp_err_t hMetrics(httpd_req_t *req) {
  if (!g_settings.metricsEnabled) {
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_sendstr(req, "# metrics disabled\n");
    return ESP_OK;
  }
  if (!metricsAuthed(req)) {
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Bearer");
    httpd_resp_sendstr(req, "# unauthorized\n");
    return ESP_OK;
  }
  WebStatus st; appGetStatus(&st);
  String m; m.reserve(1400);
  m += "# HELP hdc_temperature_celsius Temperature in Celsius\n# TYPE hdc_temperature_celsius gauge\n";
  m += "hdc_temperature_celsius " + String(st.tempC, 2) + "\n";
  m += "# HELP hdc_temperature_fahrenheit Temperature in Fahrenheit\n# TYPE hdc_temperature_fahrenheit gauge\n";
  m += "hdc_temperature_fahrenheit " + String(st.tempF, 2) + "\n";
  m += "# HELP hdc_humidity_percent Relative humidity percent\n# TYPE hdc_humidity_percent gauge\n";
  m += "hdc_humidity_percent " + String(st.humidity, 2) + "\n";
  m += "# HELP hdc_airflow_mps Air velocity estimate (m/s)\n# TYPE hdc_airflow_mps gauge\n";
  m += "hdc_airflow_mps " + String(st.airflow / 100.0, 2) + "\n";
  m += "# HELP hdc_relay_state Relay output state (0/1)\n# TYPE hdc_relay_state gauge\n";
  m += "hdc_relay_state{relay=\"1\"} " + String(st.relay1 ? 1 : 0) + "\n";
  m += "hdc_relay_state{relay=\"2\"} " + String(st.relay2 ? 1 : 0) + "\n";
  m += "# HELP hdc_contact_state Contact input state (0/1)\n# TYPE hdc_contact_state gauge\n";
  m += "hdc_contact_state{contact=\"1\"} " + String(st.contact1 ? 1 : 0) + "\n";
  m += "hdc_contact_state{contact=\"2\"} " + String(st.contact2 ? 1 : 0) + "\n";
  m += "# HELP hdc_link_up Interface link up (1=up)\n# TYPE hdc_link_up gauge\n";
  m += "hdc_link_up{iface=\"ethernet\"} " + String(st.ethLink ? 1 : 0) + "\n";
  m += "hdc_link_up{iface=\"wifi\"} " + String(st.wifiUp ? 1 : 0) + "\n";
  m += "# HELP hdc_wifi_rssi_dbm WiFi signal strength\n# TYPE hdc_wifi_rssi_dbm gauge\n";
  m += "hdc_wifi_rssi_dbm " + String(st.wifiRssi) + "\n";
  m += "# HELP hdc_uptime_seconds Uptime in seconds\n# TYPE hdc_uptime_seconds counter\n";
  m += "hdc_uptime_seconds " + String((unsigned long)(millis() / 1000)) + "\n";
  m += "# HELP hdc_free_heap_bytes Free heap memory\n# TYPE hdc_free_heap_bytes gauge\n";
  m += "hdc_free_heap_bytes " + String((unsigned long)ESP.getFreeHeap()) + "\n";
  m += "# HELP hdc_info Node information\n# TYPE hdc_info gauge\n";
  m += "hdc_info{node=\"" + promEscape(g_settings.nodeName) + "\",location=\"" + promEscape(g_settings.location) +
       "\",version=\"" SOFTWARE_VERSION_STR "\",mac=\"" + promEscape(st.mac) + "\",ip=\"" + promEscape(st.ip) + "\"} 1\n";
  httpd_resp_set_type(req, "text/plain; version=0.0.4; charset=utf-8");
  return httpd_resp_sendstr(req, m.c_str());
}

static esp_err_t hGrafana(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"hdc-sense-control-grafana.json\"");
  return httpd_resp_send(req, GRAFANA_JSON, HTTPD_RESP_USE_STRLEN);
}

// --- Downloadable integration assets ---
static esp_err_t hMib(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"HDC-SENSE-CONTROL-MIB.txt\"");
  return httpd_resp_send(req, MIB_TXT, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t hZabbix(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  // Inject the node's read community into the template's macro default.
  String yaml = String(ZABBIX_YAML);
  yaml.replace("__SNMP_COMMUNITY__", g_settings.snmpReadCommunity);
  httpd_resp_set_type(req, "application/yaml; charset=utf-8");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"hdc-sense-control-zabbix7.yaml\"");
  return httpd_resp_sendstr(req, yaml.c_str());
}

static esp_err_t hFactory(httpd_req_t *req) {
  if (!authed(req)) return denyAuth(req);
  httpd_resp_sendstr(req, "{\"ok\":true}");
  delay(300);
  settingsFactoryReset();
  appReboot();
  return ESP_OK;
}

static void reg(const char *path, httpd_method_t m, esp_err_t (*fn)(httpd_req_t *)) {
  httpd_uri_t u = {};
  u.uri = path; u.method = m; u.handler = fn; u.user_ctx = nullptr;
  httpd_register_uri_handler(s_server, &u);
}

void webBegin() {
  size_t certLen, keyLen;
  const uint8_t *cert = certsServerCert(&certLen);
  const uint8_t *key  = certsPrivateKey(&keyLen);
  if (certLen == 0 || keyLen == 0) {
    Serial.println("[WEB] No TLS cert available - HTTPS server not started.");
    return;
  }

  httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
  conf.servercert     = cert;
  conf.servercert_len = certLen;
  conf.prvtkey_pem    = key;
  conf.prvtkey_len    = keyLen;
  conf.httpd.stack_size      = 10240; // TLS handshake needs headroom
  conf.httpd.max_uri_handlers = 16;
  conf.httpd.lru_purge_enable = true;

  esp_err_t err = httpd_ssl_start(&s_server, &conf);
  if (err != ESP_OK) {
    Serial.printf("[WEB] httpd_ssl_start failed: %d\n", err);
    return;
  }

  reg("/", HTTP_GET, hRoot);
  reg("/api/login", HTTP_POST, hLogin);
  reg("/api/logout", HTTP_POST, hLogout);
  reg("/api/status", HTTP_GET, hStatus);
  reg("/api/settings", HTTP_GET, hGetSettings);
  reg("/api/settings", HTTP_POST, hSetSettings);
  reg("/api/relay", HTTP_POST, hRelay);
  reg("/api/password", HTTP_POST, hPassword);
  reg("/api/reboot", HTTP_POST, hReboot);
  reg("/api/factory-reset", HTTP_POST, hFactory);
  reg("/api/mib", HTTP_GET, hMib);
  reg("/api/zabbix", HTTP_GET, hZabbix);
  reg("/api/grafana", HTTP_GET, hGrafana);
  reg("/api/stream", HTTP_GET, hStream);
  reg("/metrics", HTTP_GET, hMetrics);

  s_started = true;
  Serial.println("[WEB] HTTPS server started on port 443.");
}

bool webStarted() { return s_started; }
