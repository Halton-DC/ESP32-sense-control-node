#include "settings.h"
#include <Preferences.h>
#include <string.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "mbedtls/pkcs5.h"
#include "mbedtls/md.h"

Settings g_settings;
char g_initialAdminPassword[24] = {0};

static Preferences prefs;
static SemaphoreHandle_t s_lock = nullptr;
static const char *NVS_NS    = "hdc";
static const char *NVS_KEY   = "settings";
static const char *NVS_RELAY = "relaybits";

void settingsLock()   { if (s_lock) xSemaphoreTakeRecursive(s_lock, portMAX_DELAY); }
void settingsUnlock() { if (s_lock) xSemaphoreGiveRecursive(s_lock); }

// Certs live in the same namespace; declared here so factory reset can wipe them.
static const char *NVS_CERT = "tlscert";
static const char *NVS_PKEY = "tlskey";

static void strlcpy_safe(char *dst, const char *src, size_t n) {
  if (n == 0) return;
  strncpy(dst, src, n - 1);
  dst[n - 1] = '\0';
}

void settingsSanitizeHostname(const char *in, char *out, size_t n) {
  size_t j = 0;
  for (size_t i = 0; in[i] && j < n - 1; i++) {
    char c = in[i];
    if (c >= 'A' && c <= 'Z') c += 32;             // lowercase
    bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-';
    if (ok) out[j++] = c;
    else if (j > 0 && out[j - 1] != '-') out[j++] = '-'; // collapse to hyphen
  }
  while (j > 0 && out[j - 1] == '-') j--;           // no trailing hyphen
  if (j == 0) { const char *f = "hdc-node"; while (*f) out[j++] = *f++; }
  out[j] = '\0';
}

void settingsLoadDefaults() {
  memset(&g_settings, 0, sizeof(g_settings));
  g_settings.magic   = SETTINGS_MAGIC;
  g_settings.version = SETTINGS_VERSION;

  strlcpy_safe(g_settings.nodeName, DEFAULT_NODE_NAME, sizeof(g_settings.nodeName));
  strlcpy_safe(g_settings.location, DEFAULT_NODE_LOCATION, sizeof(g_settings.location));
  strlcpy_safe(g_settings.contact,  DEFAULT_NODE_CONTACT,  sizeof(g_settings.contact));
  settingsSanitizeHostname(DEFAULT_NODE_NAME, g_settings.hostname, sizeof(g_settings.hostname));

  g_settings.dhcpEnabled = true;
  // Fallback static address (used if DHCP fails or is disabled).
  g_settings.staticIp   = (uint32_t)IPAddress(192, 168, 9, 200);
  g_settings.staticMask = (uint32_t)IPAddress(255, 255, 255, 0);
  g_settings.staticGw   = (uint32_t)IPAddress(192, 168, 9, 1);
  g_settings.staticDns  = (uint32_t)IPAddress(192, 168, 9, 1);

  strlcpy_safe(g_settings.wifiSsid, DEFAULT_WIFI_SSID, sizeof(g_settings.wifiSsid));
  strlcpy_safe(g_settings.wifiPass, DEFAULT_WIFI_PASS, sizeof(g_settings.wifiPass));
  g_settings.wifiEnabled = strlen(DEFAULT_WIFI_SSID) > 0;
  strlcpy_safe(g_settings.apPassword, "hdc-setup", sizeof(g_settings.apPassword));

  g_settings.snmpEnabled      = true;
  g_settings.snmpWriteEnabled = true;
  strlcpy_safe(g_settings.snmpReadCommunity,  DEFAULT_SNMP_READ,  sizeof(g_settings.snmpReadCommunity));
  strlcpy_safe(g_settings.snmpWriteCommunity, DEFAULT_SNMP_WRITE, sizeof(g_settings.snmpWriteCommunity));

  strlcpy_safe(g_settings.adminUser, DEFAULT_ADMIN_USER, sizeof(g_settings.adminUser));
  {
    char pw[24];
    if (strlen(DEFAULT_ADMIN_PASSWORD) > 0) {
      strlcpy_safe(pw, DEFAULT_ADMIN_PASSWORD, sizeof(pw));
    } else {
      // Strong random factory password (ambiguity-free charset).
      static const char *cs = "ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnpqrstuvwxyz23456789";
      int L = strlen(cs);
      uint8_t b[12];
      esp_fill_random(b, sizeof(b));
      for (int i = 0; i < 12; i++) pw[i] = cs[b[i] % L];
      pw[12] = '\0';
    }
    settingsSetPassword(pw);
    strlcpy_safe(g_initialAdminPassword, pw, sizeof(g_initialAdminPassword));
  }
  g_settings.mustChangePassword = true;

  strlcpy_safe(g_settings.relay1Name, "Relay 1", sizeof(g_settings.relay1Name));
  strlcpy_safe(g_settings.relay2Name, "Relay 2", sizeof(g_settings.relay2Name));
  g_settings.relay1State = false;
  g_settings.relay2State = false;

  g_settings.sensorIntervalMs = 1000;

  g_settings.metricsEnabled = true;
  g_settings.metricsToken[0] = '\0';   // open by default (set a token to require Bearer auth)
}

void settingsBegin() {
  if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutex();
  prefs.begin(NVS_NS, false);
  size_t got = prefs.getBytesLength(NVS_KEY);
  bool valid = false;
  if (got == sizeof(Settings)) {
    Settings tmp;
    prefs.getBytes(NVS_KEY, &tmp, sizeof(tmp));
    if (tmp.magic == SETTINGS_MAGIC && tmp.version == SETTINGS_VERSION) {
      g_settings = tmp;
      valid = true;
    }
  }
  if (!valid) {
    Serial.println("[CFG] No valid settings found - writing factory defaults.");
    settingsLoadDefaults();
    settingsSave();
  } else {
    Serial.println("[CFG] Settings loaded from NVS.");
  }
  // Relay state persists in its own small key (authoritative if present).
  if (prefs.isKey(NVS_RELAY)) {
    uint8_t b = prefs.getUChar(NVS_RELAY, 0);
    g_settings.relay1State = b & 0x01;
    g_settings.relay2State = b & 0x02;
  }
}

bool settingsSave() {
  settingsLock();
  g_settings.magic   = SETTINGS_MAGIC;
  g_settings.version = SETTINGS_VERSION;
  size_t w = prefs.putBytes(NVS_KEY, &g_settings, sizeof(g_settings));
  settingsUnlock();
  return w == sizeof(g_settings);
}

void settingsSaveRelays() {
  settingsLock();
  uint8_t b = (g_settings.relay1State ? 0x01 : 0) | (g_settings.relay2State ? 0x02 : 0);
  prefs.putUChar(NVS_RELAY, b);
  settingsUnlock();
}

void settingsFactoryReset() {
  Serial.println("[CFG] FACTORY RESET - erasing all stored configuration.");
  settingsLock();
  prefs.remove(NVS_KEY);
  prefs.remove(NVS_RELAY);
  prefs.remove(NVS_CERT); // force regeneration of the TLS certificate
  prefs.remove(NVS_PKEY);
  settingsLoadDefaults();
  settingsSave();
  settingsUnlock();
}

// --- Password hashing (PBKDF2-HMAC-SHA256) ---

static void pbkdf2(const char *password, const uint8_t *salt, uint8_t *out) {
  // mbedtls 3.x: pkcs5_pbkdf2_hmac_ext takes the md type directly.
  mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256,
                                (const unsigned char *)password, strlen(password),
                                salt, PWD_SALT_LEN,
                                PBKDF2_ITERS, PWD_HASH_LEN, out);
}

void settingsSetPassword(const char *password) {
  esp_fill_random(g_settings.adminSalt, PWD_SALT_LEN);
  pbkdf2(password, g_settings.adminSalt, g_settings.adminHash);
}

bool settingsVerifyPassword(const char *password) {
  uint8_t calc[PWD_HASH_LEN];
  pbkdf2(password, g_settings.adminSalt, calc);
  // Constant-time comparison.
  uint8_t diff = 0;
  for (int i = 0; i < PWD_HASH_LEN; i++) diff |= (calc[i] ^ g_settings.adminHash[i]);
  return diff == 0;
}
