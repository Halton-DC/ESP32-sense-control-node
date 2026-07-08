/**
 * HDC Sense & Control Node - Data Center Environmental Monitor
 * -----------------------------------------------------------
 * ESP32-S3-ETH (W5500) node deployed across the data center to report
 * temperature, humidity and airflow, with authenticated remote control.
 *
 *   Monitoring : SNMPv2c (private community)  -> Zabbix
 *   Management : HTTPS web UI + authenticated REST API (TLS)
 *   Network    : DHCP with static fallback (configurable)
 *   Config     : persisted in NVS; factory-resettable (BOOT button or REST)
 *
 * Stack: arduino-esp32 3.x, W5500 as an lwIP ETH interface.
 * See HDC-SENSE-CONTROL-MIB.txt for the SNMP OID tree.
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ETH.h>
#include <WiFi.h>
#include <NetworkUdp.h>
#include <Adafruit_SHT31.h>
#include <SNMP_Agent.h>

#include "settings.h"
#include "certs.h"
#include "web.h"

// ============================================================
//  HARDWARE PINS
// ============================================================
#define RELAY1_PIN    36
#define RELAY2_PIN    37
#define CONTACT1_PIN  43
#define CONTACT2_PIN  44
#define AIRFLOW_PIN   1
#define SDA_PIN       33
#define SCL_PIN       34
#define BOOT_BTN_PIN  0     // hold at power-up for factory reset

// W5500 SPI
#define W5500_CS   14
#define W5500_RST   9
#define W5500_INT  10
#define W5500_MISO 12
#define W5500_MOSI 11
#define W5500_SCK  13

// ============================================================
//  SNMP OID MAP (see HDC-SENSE-CONTROL-MIB.txt) - base .1.3.6.1.4.1.55555.1
// ============================================================
#define OID_TEMP_C     ".1.3.6.1.4.1.55555.1.1.0"
#define OID_TEMP_F     ".1.3.6.1.4.1.55555.1.2.0"
#define OID_HUMIDITY   ".1.3.6.1.4.1.55555.1.3.0"
#define OID_AIRFLOW    ".1.3.6.1.4.1.55555.1.4.0"
#define OID_RELAY1     ".1.3.6.1.4.1.55555.1.5.0"
#define OID_RELAY2     ".1.3.6.1.4.1.55555.1.6.0"
#define OID_CONTACT1   ".1.3.6.1.4.1.55555.1.7.0"
#define OID_CONTACT2   ".1.3.6.1.4.1.55555.1.8.0"
#define OID_UPTIME     ".1.3.6.1.4.1.55555.1.9.0"
#define OID_NODE_NAME  ".1.3.6.1.4.1.55555.1.10.0"
#define OID_BOARD_INFO ".1.3.6.1.4.1.55555.1.11.0"
#define OID_CHIP_INFO  ".1.3.6.1.4.1.55555.1.12.0"
#define OID_MAC_ADDR   ".1.3.6.1.4.1.55555.1.13.0"
#define OID_FLASH_SIZE ".1.3.6.1.4.1.55555.1.14.0"
#define OID_SW_VERSION ".1.3.6.1.4.1.55555.1.15.0"
// MIB-II system group
#define OID_SYS_DESCR    ".1.3.6.1.2.1.1.1.0"
#define OID_SYS_UPTIME   ".1.3.6.1.2.1.1.3.0"
#define OID_SYS_CONTACT  ".1.3.6.1.2.1.1.4.0"
#define OID_SYS_NAME     ".1.3.6.1.2.1.1.5.0"
#define OID_SYS_LOCATION ".1.3.6.1.2.1.1.6.0"

// ============================================================
//  GLOBALS
// ============================================================
Adafruit_SHT31 sht31 = Adafruit_SHT31();
NetworkUDP     snmpUdp;
SNMPAgent     *snmp = nullptr;

bool netUp        = false;
bool snmpRunning  = false;
bool sensorOk     = false;
bool apActive     = false;   // SoftAP provisioning fallback is up
char apSsid[24]   = {0};

// SNMP-exposed integer values (agent holds pointers to these).
int metricTempC100 = 0, metricTempF100 = 0, metricHum100 = 0;
int metricAirflow  = 0, metricC1 = 0, metricC2 = 0;
int relayState[2]  = {0, 0};        // runtime source of truth (0/1)
int lastRelay[2]   = {-1, -1};
int snmpUptime     = 0;             // TimeTicks (1/100 s); wraps ~248 days (lib API is int*)
int flashSizeB     = 0;

// Static descriptive strings (built once at boot).
std::string macStr, chipInfo, boardInfo = "ESP32-S3-ETH + W5500", sysDescr;

// ============================================================
//  HELPERS
// ============================================================
static void formatUptime(char *buf, size_t n) {
  unsigned long s = millis() / 1000;
  unsigned long d = s / 86400; s %= 86400;
  unsigned long h = s / 3600;  s %= 3600;
  unsigned long m = s / 60;    s %= 60;
  if (d) snprintf(buf, n, "%lud %02lu:%02lu:%02lu", d, h, m, s);
  else   snprintf(buf, n, "%02lu:%02lu:%02lu", h, m, s);
}

static void applyRelayPin(int idx) {
  digitalWrite(idx == 0 ? RELAY1_PIN : RELAY2_PIN, relayState[idx] ? HIGH : LOW);
}

static void buildStaticInfo() {
  uint8_t m[6];
  ETH.macAddress(m);
  char b[24];
  snprintf(b, sizeof(b), "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
  macStr = b;

  char c[96];
  snprintf(c, sizeof(c), "ESP32 %s rev%d, %d core(s)",
           ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());
  chipInfo = c;
  flashSizeB = (int)ESP.getFlashChipSize();
  sysDescr = std::string("HDC Sense & Control Node v") + SOFTWARE_VERSION_STR +
             " (" + boardInfo + ")";
}

// ============================================================
//  NETWORK  (Ethernet + WiFi, DHCP/static, SoftAP fallback)
//  Ethernet and WiFi are both lwIP interfaces, so the HTTPS server and
//  SNMP agent serve on whichever interface(s) have an address.
// ============================================================
bool ethUp  = false;
bool wifiUp = false;

static void setupEthernet() {
  Serial.println("[ETH] Bringing up W5500...");
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW); delay(50);
  digitalWrite(W5500_RST, HIGH); delay(200);

  SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);
  ETH.setHostname(g_settings.hostname);   // announced to DHCP
  if (!ETH.begin(ETH_PHY_W5500, 1, W5500_CS, W5500_INT, W5500_RST, SPI)) {
    Serial.println("[ETH] ETH.begin() failed (W5500 not detected).");
    return;
  }

  IPAddress ip(g_settings.staticIp), gw(g_settings.staticGw),
            mask(g_settings.staticMask), dns(g_settings.staticDns);

  if (!g_settings.dhcpEnabled) {
    Serial.println("[ETH] DHCP disabled - configuring static IP.");
    ETH.config(ip, gw, mask, dns);
  }

  uint32_t t0 = millis();
  while (!ETH.linkUp() && millis() - t0 < 5000) delay(100);
  if (!ETH.linkUp()) { Serial.println("[ETH] No link (no cable?)."); return; }

  if (g_settings.dhcpEnabled) {
    Serial.println("[ETH] Requesting DHCP lease...");
    t0 = millis();
    while ((uint32_t)ETH.localIP() == 0 && millis() - t0 < 10000) delay(100);
    if ((uint32_t)ETH.localIP() == 0) {
      Serial.println("[ETH] DHCP failed - static fallback.");
      ETH.config(ip, gw, mask, dns);
    }
  }
  ethUp = (uint32_t)ETH.localIP() != 0;
  if (ethUp) { Serial.print("[ETH] IP: "); Serial.println(ETH.localIP()); }
}

static void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
    Serial.printf("[WiFi] Disconnected (reason %d)\n", info.wifi_sta_disconnected.reason);
  else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP)
    Serial.print("[WiFi] Got IP: "), Serial.println(WiFi.localIP());
}

static const char *authModeStr(int m) {
  switch (m) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-ENT";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    default: return "?";
  }
}

// Diagnostic: list visible APs (security + signal) - helps triage join failures.
static void scanNetworks() {
  Serial.println("[WiFi] Scanning for visible networks...");
  int n = WiFi.scanNetworks();
  if (n <= 0) { Serial.println("[WiFi]   (none found)"); return; }
  for (int i = 0; i < n; i++) {
    Serial.printf("[WiFi]   %-24s  %4d dBm  ch%-2d  %s\n",
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                  authModeStr(WiFi.encryptionType(i)));
  }
  WiFi.scanDelete();
}

static void setupWifiStation() {
  if (!g_settings.wifiEnabled || strlen(g_settings.wifiSsid) == 0) return;
  Serial.printf("[WiFi] Connecting to \"%s\" (2.4GHz only)...\n", g_settings.wifiSsid);
  WiFi.onEvent(onWifiEvent);
  WiFi.setHostname(g_settings.hostname);   // announced to DHCP
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(g_settings.wifiSsid, g_settings.wifiPass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(200);
  if (WiFi.status() == WL_CONNECTED) {
    wifiUp = true;
    Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());
    Serial.printf("[WiFi] RSSI: %d dBm, host: %s\n", WiFi.RSSI(), WiFi.getHostname());
  } else {
    Serial.println("[WiFi] Connection failed (check band/credentials).");
    scanNetworks();
  }
}

// Raise a WPA2 setup access point so the node is always reachable for config.
static void startSetupAp() {
  uint8_t m[6];
  ETH.macAddress(m);
  snprintf(apSsid, sizeof(apSsid), "HDC-Node-%02X%02X", m[4], m[5]);
  WiFi.mode(wifiUp ? WIFI_AP_STA : WIFI_AP);
  bool ok = WiFi.softAP(apSsid, g_settings.apPassword);
  if (ok) {
    apActive = true;
    Serial.printf("[WiFi] Setup AP \"%s\" up at %s (pass: %s)\n",
                  apSsid, WiFi.softAPIP().toString().c_str(), g_settings.apPassword);
  } else {
    Serial.println("[WiFi] Failed to start setup AP.");
  }
}

static void setupNetwork() {
  setupEthernet();
  setupWifiStation();
  netUp = ethUp || wifiUp;
  if (!netUp) {
    Serial.println("[NET] No wired/WiFi connection - starting setup AP.");
    startSetupAp();
  }
}

// Prefer Ethernet whenever it has link + an address (incl. runtime hot-plug);
// otherwise fall back to WiFi. WiFi stays associated as a warm backup.
// Servers bind to all interfaces; this only sets the default outbound route.
static void manageNetworkPreference() {
  static int preferred = -1; // -1 none, 0 Ethernet, 1 WiFi
  bool ethReady  = ETH.linkUp() && (uint32_t)ETH.localIP() != 0;
  bool wifiReady = (WiFi.status() == WL_CONNECTED);

  if (ethReady && preferred != 0) {
    ETH.setDefault();
    preferred = 0; ethUp = true;
    Serial.print("[NET] Ethernet preferred (default route): ");
    Serial.println(ETH.localIP());
  } else if (!ethReady && wifiReady && preferred != 1) {
    WiFi.STA.setDefault();
    preferred = 1; ethUp = false;
    Serial.print("[NET] WiFi preferred (default route): ");
    Serial.println(WiFi.localIP());
  } else if (!ethReady && !wifiReady) {
    preferred = -1; ethUp = false;
  }
}

// ============================================================
//  SNMP
// ============================================================
static void setupSnmp() {
  if (!g_settings.snmpEnabled || !netUp) {
    Serial.println("[SNMP] Disabled or no network - agent not started.");
    return;
  }
  snmp = new SNMPAgent(g_settings.snmpReadCommunity, g_settings.snmpWriteCommunity);
  snmpUdp.begin(161);
  snmp->setUDP(&snmpUdp);
  snmp->begin();

  // MIB-II system group
  snmp->addReadOnlyStaticStringHandler((char *)OID_SYS_DESCR, sysDescr);
  snmp->addTimestampHandler((char *)OID_SYS_UPTIME, &snmpUptime);
  snmp->addReadOnlyStaticStringHandler((char *)OID_SYS_CONTACT, std::string(g_settings.contact));
  snmp->addReadOnlyStaticStringHandler((char *)OID_SYS_NAME, std::string(g_settings.nodeName));
  snmp->addReadOnlyStaticStringHandler((char *)OID_SYS_LOCATION, std::string(g_settings.location));

  // Environmental
  snmp->addIntegerHandler((char *)OID_TEMP_C, &metricTempC100);
  snmp->addIntegerHandler((char *)OID_TEMP_F, &metricTempF100);
  snmp->addIntegerHandler((char *)OID_HUMIDITY, &metricHum100);
  snmp->addIntegerHandler((char *)OID_AIRFLOW, &metricAirflow);

  // Relays: settable only if SNMP writes are allowed.
  snmp->addIntegerHandler((char *)OID_RELAY1, &relayState[0], g_settings.snmpWriteEnabled);
  snmp->addIntegerHandler((char *)OID_RELAY2, &relayState[1], g_settings.snmpWriteEnabled);
  snmp->addIntegerHandler((char *)OID_CONTACT1, &metricC1);
  snmp->addIntegerHandler((char *)OID_CONTACT2, &metricC2);

  // Diagnostics
  snmp->addTimestampHandler((char *)OID_UPTIME, &snmpUptime);
  snmp->addReadOnlyStaticStringHandler((char *)OID_NODE_NAME, std::string(g_settings.nodeName));
  snmp->addReadOnlyStaticStringHandler((char *)OID_BOARD_INFO, boardInfo);
  snmp->addReadOnlyStaticStringHandler((char *)OID_CHIP_INFO, chipInfo);
  snmp->addReadOnlyStaticStringHandler((char *)OID_MAC_ADDR, macStr);
  snmp->addIntegerHandler((char *)OID_FLASH_SIZE, &flashSizeB);
  snmp->addReadOnlyStaticStringHandler((char *)OID_SW_VERSION, (char *)SOFTWARE_VERSION_STR);

  snmp->sortHandlers();
  snmpRunning = true;
  Serial.printf("[SNMP] Agent up (writes %s).\n",
                g_settings.snmpWriteEnabled ? "enabled" : "disabled");
}

// ============================================================
//  SENSORS
// ============================================================
static void refreshSensors() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if (!isnan(t) && !isnan(h)) {
    metricTempC100 = (int)lroundf(t * 100.0f);
    metricTempF100 = (int)lroundf((t * 1.8f + 32.0f) * 100.0f);
    metricHum100   = (int)lroundf(h * 100.0f);
    sensorOk = true;
  } else {
    sensorOk = false;
  }
  metricAirflow = analogRead(AIRFLOW_PIN);
  metricC1 = digitalRead(CONTACT1_PIN);
  metricC2 = digitalRead(CONTACT2_PIN);
  snmpUptime = millis() / 10;
}

// ============================================================
//  FACTORY RESET (BOOT button held at startup)
// ============================================================
static void checkFactoryButton() {
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
  if (digitalRead(BOOT_BTN_PIN) != LOW) return;
  Serial.println("[SYS] BOOT held - hold 5s for factory reset...");
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    if (digitalRead(BOOT_BTN_PIN) != LOW) return; // released -> cancel
    delay(50);
  }
  Serial.println("[SYS] Factory reset triggered by BOOT button.");
  settingsFactoryReset();
  ESP.restart();
}

// ============================================================
//  APP INTERFACE (called by web.cpp)
// ============================================================
void appGetStatus(WebStatus *s) {
  s->tempC = metricTempC100 / 100.0f;
  s->tempF = metricTempF100 / 100.0f;
  s->humidity = metricHum100 / 100.0f;
  s->airflow = metricAirflow;
  s->contact1 = metricC1; s->contact2 = metricC2;
  s->relay1 = relayState[0]; s->relay2 = relayState[1];

  s->ethLink = ETH.linkUp();
  strncpy(s->ethIp, ETH.localIP().toString().c_str(), sizeof(s->ethIp) - 1);
  s->ethIp[sizeof(s->ethIp) - 1] = 0;

  s->wifiUp = (WiFi.status() == WL_CONNECTED);
  strncpy(s->wifiIp, WiFi.localIP().toString().c_str(), sizeof(s->wifiIp) - 1);
  s->wifiIp[sizeof(s->wifiIp) - 1] = 0;
  s->wifiRssi = s->wifiUp ? WiFi.RSSI() : 0;
  s->apActive = apActive;

  // Primary interface for the header + link pill.
  #define CPY(dst, src) do { strncpy((dst), (src), sizeof(dst) - 1); (dst)[sizeof(dst) - 1] = 0; } while (0)
  if (s->ethLink && (uint32_t)ETH.localIP() != 0) {
    CPY(s->mode, "Ethernet"); CPY(s->ip, s->ethIp); s->link = true; CPY(s->ssid, "");
  } else if (s->wifiUp) {
    CPY(s->mode, "WiFi"); CPY(s->ip, s->wifiIp); s->link = true; CPY(s->ssid, g_settings.wifiSsid);
  } else if (apActive) {
    CPY(s->mode, "Setup AP"); CPY(s->ip, WiFi.softAPIP().toString().c_str()); s->link = false; CPY(s->ssid, apSsid);
  } else {
    CPY(s->mode, "Offline"); CPY(s->ip, "0.0.0.0"); s->link = false; CPY(s->ssid, "");
  }
  #undef CPY
  strncpy(s->mac, macStr.c_str(), sizeof(s->mac) - 1); s->mac[sizeof(s->mac) - 1] = 0;
  formatUptime(s->uptime, sizeof(s->uptime));
}

void appSetRelay(int idx, bool on) {
  if (idx < 1 || idx > 2) return;
  settingsLock();
  relayState[idx - 1] = on ? 1 : 0;
  lastRelay[idx - 1] = relayState[idx - 1];
  applyRelayPin(idx - 1);
  (idx == 1 ? g_settings.relay1State : g_settings.relay2State) = on;
  settingsSaveRelays();
  settingsUnlock();
  Serial.printf("[RELAY] %d -> %s\n", idx, on ? "ON" : "OFF");
}

void appReboot() { ESP.restart(); }

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[SYS] HDC Sense & Control Node booting (v" SOFTWARE_VERSION_STR ")");

  checkFactoryButton();
  settingsBegin();

  // Print the one-time factory admin password (only when freshly generated).
  if (g_settings.mustChangePassword && g_initialAdminPassword[0]) {
    Serial.println("\n============================================================");
    Serial.printf ("  INITIAL ADMIN LOGIN\n  user: %s\n  pass: %s\n",
                   g_settings.adminUser, g_initialAdminPassword);
    Serial.println("  You will be required to change this at first login.");
    Serial.println("============================================================\n");
  }

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(CONTACT1_PIN, INPUT_PULLUP);
  pinMode(CONTACT2_PIN, INPUT_PULLUP);
  relayState[0] = g_settings.relay1State ? 1 : 0;
  relayState[1] = g_settings.relay2State ? 1 : 0;
  applyRelayPin(0); applyRelayPin(1);
  lastRelay[0] = relayState[0]; lastRelay[1] = relayState[1];

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!sht31.begin(0x44)) Serial.println("[SENSOR] SHT31 not found at 0x44!");
  else                    Serial.println("[SENSOR] SHT31 ready.");

  setupNetwork();
  buildStaticInfo();
  Serial.print("[SYS] MAC: "); Serial.println(macStr.c_str());

  if (netUp) setupSnmp();  // SNMP only over real (routed) networks
  if (netUp || apActive) {
    if (certsBegin(g_settings.nodeName)) webBegin();
    else Serial.println("[WEB] TLS unavailable - web UI disabled.");
  } else {
    Serial.println("[SYS] No network - running in local sensor mode.");
  }

  refreshSensors();
  Serial.println("[SYS] Node initialized.");
}

void loop() {
  if (snmpRunning) snmp->loop();

  // Reconcile relay changes originating from SNMP SET (normalize to 0/1 so a
  // GET after a non-boolean SET still reads back 0/1).
  for (int i = 0; i < 2; i++) {
    int want = relayState[i] ? 1 : 0;
    relayState[i] = want;
    if (want != lastRelay[i]) {
      settingsLock();
      lastRelay[i] = want;
      applyRelayPin(i);
      (i == 0 ? g_settings.relay1State : g_settings.relay2State) = want;
      settingsSaveRelays();
      settingsUnlock();
      Serial.printf("[RELAY] %d -> %s (via SNMP)\n", i + 1, want ? "ON" : "OFF");
    }
  }

  static uint32_t lastRefresh = 0, lastDebug = 0, lastNet = 0;
  uint32_t now = millis();
  if (now - lastRefresh >= g_settings.sensorIntervalMs) {
    lastRefresh = now;
    refreshSensors();
  }
  if (now - lastNet >= 2000) {
    lastNet = now;
    manageNetworkPreference();
  }
  if (now - lastDebug >= 5000) {
    lastDebug = now;
    WebStatus st; appGetStatus(&st);
    Serial.printf("[STAT] %s %s | T=%.2fC H=%.2f%% Air=%d C1=%d C2=%d R1=%d R2=%d%s\n",
                  st.mode, st.ip,
                  metricTempC100 / 100.0f, metricHum100 / 100.0f, metricAirflow,
                  metricC1, metricC2, relayState[0], relayState[1],
                  sensorOk ? "" : " [SENSOR ERR]");
  }
}
