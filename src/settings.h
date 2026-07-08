/**
 * settings.h - Persistent configuration for the HDC Sense & Control Node.
 *
 * All node configuration lives in a single versioned struct stored in NVS
 * (flash). Passwords are never stored in clear text: only a random salt and
 * a PBKDF2-HMAC-SHA256 hash are kept. The TLS certificate/key are stored as
 * separate NVS blobs (see certs.*).
 */
#pragma once
#include <Arduino.h>
#include <stdint.h>

#define SOFTWARE_VERSION_STR "2.0.0"
#ifndef BUILD_NUMBER
#define BUILD_NUMBER 0
#endif

// Compile-time defaults (override per fleet via build_flags).
#ifndef DEFAULT_NODE_NAME
#define DEFAULT_NODE_NAME "" // empty => hdc-sense-<last 2 bytes of Ethernet MAC>
#endif
#ifndef DEFAULT_NODE_LOCATION
#define DEFAULT_NODE_LOCATION "DC1 / Row A / Rack 01"
#endif
#ifndef DEFAULT_NODE_CONTACT
#define DEFAULT_NODE_CONTACT "Halton Datacenter NOC"
#endif
#ifndef DEFAULT_ADMIN_USER
#define DEFAULT_ADMIN_USER "admin"
#endif
#ifndef DEFAULT_ADMIN_PASSWORD
#define DEFAULT_ADMIN_PASSWORD "" // empty => strong random per-device password
#endif
// Optional default WiFi station credentials (set via build_flags for dev).
#ifndef DEFAULT_WIFI_SSID
#define DEFAULT_WIFI_SSID ""
#endif
#ifndef DEFAULT_WIFI_PASS
#define DEFAULT_WIFI_PASS ""
#endif

// Private SNMP communities by default (no "public"/"private" well-known names).
#ifndef DEFAULT_SNMP_READ
#define DEFAULT_SNMP_READ "hdc-ro-9f3a"
#endif
#ifndef DEFAULT_SNMP_WRITE
#define DEFAULT_SNMP_WRITE "hdc-rw-2b7c"
#endif

#define SETTINGS_MAGIC   0x48444332UL // "HDC2"
#define SETTINGS_VERSION 6
#define PWD_SALT_LEN     16
#define PWD_HASH_LEN     32
#define PBKDF2_ITERS     20000

struct Settings {
  uint32_t magic;
  uint16_t version;

  // --- Identity ---
  char nodeName[32];
  char location[64];
  char contact[64];
  char hostname[32];   // DHCP/mDNS hostname (shown in the router lease list)

  // --- Ethernet (W5500) ---
  bool     dhcpEnabled;      // try DHCP first
  uint32_t staticIp;         // fallback / static address (network byte order via IPAddress)
  uint32_t staticMask;
  uint32_t staticGw;
  uint32_t staticDns;

  // --- WiFi (station) ---
  bool wifiEnabled;
  char wifiSsid[33];
  char wifiPass[64];
  // SoftAP provisioning fallback (raised when no network is reachable)
  char apPassword[64];       // WPA2 passphrase for the setup AP

  // --- SNMP ---
  bool snmpEnabled;
  bool snmpWriteEnabled;     // allow relay writes via the write community
  char snmpReadCommunity[32];
  char snmpWriteCommunity[32];

  // --- Web / REST admin credentials ---
  char          adminUser[32];
  uint8_t       adminSalt[PWD_SALT_LEN];
  uint8_t       adminHash[PWD_HASH_LEN];
  bool          mustChangePassword; // true until the default password is changed

  // --- Relays ---
  char relay1Name[24];
  char relay2Name[24];
  bool relay1State;          // desired state, restored on boot
  bool relay2State;

  // --- Sensors / display ---
  uint16_t sensorIntervalMs; // sampling cadence
  bool     tempUnitF;        // dashboard primary temperature unit: false=°C, true=°F
  int16_t  tempOffC10;       // temperature calibration offset, 0.1 °C units
  int16_t  humOff10;         // humidity calibration offset, 0.1 %RH units
  int16_t  airflowOff;       // airflow calibration offset, raw ADC counts

  // --- Prometheus /metrics ---
  bool metricsEnabled;
  char metricsToken[40];     // empty => open (trusted VLAN); else Bearer token
};

// Global settings instance (defined in settings.cpp).
extern Settings g_settings;

// Plaintext initial admin password, populated ONLY when defaults are generated
// (first boot / factory reset) so it can be printed once to the serial console.
// Empty on normal boots. Never persisted.
extern char g_initialAdminPassword[24];

// Load settings from NVS; writes+returns defaults if absent/corrupt/outdated.
void settingsBegin();

// Persist the current g_settings to NVS. Returns true on success.
// Thread-safe (takes the settings lock internally).
bool settingsSave();

// Persist only the relay states to a dedicated small NVS key. Used for the
// frequently-changing relay outputs to avoid rewriting the whole blob (flash
// wear). Thread-safe.
void settingsSaveRelays();

// Recursive lock guarding g_settings + NVS. Wrap compound read-modify-write
// sequences that span multiple fields so they don't interleave with the other
// task (the HTTPS server runs in its own FreeRTOS task).
void settingsLock();
void settingsUnlock();

// Reset g_settings to factory defaults and persist (also wipes TLS cert/creds).
void settingsFactoryReset();

// Populate g_settings with factory defaults in RAM (does not persist).
void settingsLoadDefaults();

// Convert an arbitrary string into a valid DNS hostname (a-z 0-9 -).
void settingsSanitizeHostname(const char *in, char *out, size_t n);

// --- Password helpers (PBKDF2-HMAC-SHA256) ---
// Hash `password` with a freshly generated random salt into g_settings.
void settingsSetPassword(const char *password);
// Constant-time verify `password` against the stored salt+hash.
bool settingsVerifyPassword(const char *password);
