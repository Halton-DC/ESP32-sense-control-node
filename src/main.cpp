/**
 * HDC Sense & Control Node - Unified ESP32-ETH SNMP Firmware
 * ----------------------------------------------------------
 * Board: ESP32-ETH (e.g., Olimex ESP32-POE)
 * Sensors:
 *   - Temperature & Humidity: SHT31 (I²C @ 0x44)
 *   - Airflow: Omron D6F-V03A1 (analog voltage on pin 34)
 *   - Contact inputs: dry contacts on pins 32 & 33
 *   - Relay outputs: GPIO 25 and 26
 * Networking:
 *   - Ethernet with static IP first, DHCP fallback
 *   - SNMPv2c agent with read/write support
 * OID Tree:
 *   - See HDC-SENSE-CONTROL-MIB
 */

#include <Arduino.h>
#include <Wire.h>
#include <ETH.h>
#include <Adafruit_SHT31.h>
#include <WiFiUdp.h>
#include <esp_system.h>  // For chip info

// === Software Version ===
#define SOFTWARE_VERSION "1.0.0"

// === SNMP Community Strings ===
#define SNMP_COMMUNITY_READ  "public"
#define SNMP_COMMUNITY_WRITE "private"

// === Ethernet IP Configuration ===
const IPAddress STATIC_IP(192, 168, 1, 150);
const IPAddress GATEWAY_IP(192, 168, 1, 1);
const IPAddress SUBNET_MASK(255, 255, 255, 0);
const IPAddress DNS_IP(8, 8, 8, 8);

// === GPIO Pins ===
#define RELAY1_PIN    25  // Relay output 1
#define RELAY2_PIN    26  // Relay output 2
#define CONTACT1_PIN  32  // Contact input 1
#define CONTACT2_PIN  33  // Contact input 2
#define AIRFLOW_PIN   34  // Analog airflow input
#define SCL_PIN       22  // I²C clock
#define SDA_PIN       21  // I²C data

// === SNMP OIDs ===
#define OID_TEMP_C        "1.3.6.1.4.1.55555.1.1.0"
#define OID_TEMP_F        "1.3.6.1.4.1.55555.1.1.1"
#define OID_HUMIDITY      "1.3.6.1.4.1.55555.1.2.0"
#define OID_AIRFLOW       "1.3.6.1.4.1.55555.1.3.0"
#define OID_RELAY1        "1.3.6.1.4.1.55555.1.4.0"
#define OID_RELAY2        "1.3.6.1.4.1.55555.1.5.0"
#define OID_CONTACT1      "1.3.6.1.4.1.55555.1.6.0"
#define OID_CONTACT2      "1.3.6.1.4.1.55555.1.7.0"
#define OID_UPTIME        "1.3.6.1.4.1.55555.1.8.0"
#define OID_NODE_NAME     "1.3.6.1.4.1.55555.1.9.0"
#define OID_BOARD_INFO    "1.3.6.1.4.1.55555.1.10.0"
#define OID_CHIP_INFO     "1.3.6.1.4.1.55555.1.11.0"
#define OID_MAC_ADDR      "1.3.6.1.4.1.55555.1.12.0"
#define OID_FLASH_SIZE    "1.3.6.1.4.1.55555.1.13.0"
#define OID_SW_VERSION    "1.3.6.1.4.1.55555.1.14.0"

// === Globals ===
Adafruit_SHT31 sht31 = Adafruit_SHT31();
WiFiUDP snmpUdp;
unsigned long bootMillis = 0;
String nodeName = "hdc-node-001";

// === Ethernet Event Callback ===
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[ETH] IP: ");
      Serial.println(ETH.localIP());
      break;
    default: break;
  }
}

// === Ethernet Init (static IP first, fallback to DHCP) ===
void setupEthernet() {
  WiFi.onEvent(WiFiEvent);
  Serial.println("[ETH] Configuring static IP...");
  ETH.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, DNS_IP);
  ETH.begin();
  delay(1500);

  if (!ETH.linkUp() || ETH.localIP() == IPAddress(0, 0, 0, 0)) {
    Serial.println("[ETH] Static IP failed. Trying DHCP...");
    ETH.config(IPAddress(0, 0, 0, 0), GATEWAY_IP, SUBNET_MASK, DNS_IP);
    ETH.begin();
    delay(1500);
  }

  Serial.print("[ETH] Final IP: ");
  Serial.println(ETH.localIP());
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  setupEthernet();

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(CONTACT1_PIN, INPUT_PULLUP);
  pinMode(CONTACT2_PIN, INPUT_PULLUP);

  if (!sht31.begin(0x44)) {
    Serial.println("[SENSOR] SHT31 not found!");
  } else {
    Serial.println("[SENSOR] SHT31 ready.");
  }

  snmpUdp.begin(161);
  bootMillis = millis();

  Serial.println("[SYSTEM] Node initialized.");
}

// === SNMP GET/SET Handler ===
void handleSnmp() {
  char packet[1500];
  int len = snmpUdp.parsePacket();
  if (len > 0) {
    snmpUdp.read(packet, len);

    // === Temperature ===
    if (strstr(packet, OID_TEMP_C)) {
      sprintf(packet, "%d", (int)(sht31.readTemperature() * 100));
    } else if (strstr(packet, OID_TEMP_F)) {
      float tempF = sht31.readTemperature() * 1.8 + 32;
      sprintf(packet, "%d", (int)(tempF * 100));
    }

    // === Humidity ===
    else if (strstr(packet, OID_HUMIDITY)) {
      sprintf(packet, "%d", (int)(sht31.readHumidity() * 100));
    }

    // === Airflow ===
    else if (strstr(packet, OID_AIRFLOW)) {
      int raw = analogRead(AIRFLOW_PIN);
      sprintf(packet, "%d", raw);
    }

    // === Contact Sensors ===
    else if (strstr(packet, OID_CONTACT1)) {
      sprintf(packet, "%d", digitalRead(CONTACT1_PIN));
    } else if (strstr(packet, OID_CONTACT2)) {
      sprintf(packet, "%d", digitalRead(CONTACT2_PIN));
    }

    // === Relay Control (write) ===
    else if (strstr(packet, OID_RELAY1)) {
      if (strstr(packet, SNMP_COMMUNITY_WRITE) && strstr(packet, "SET=1")) digitalWrite(RELAY1_PIN, HIGH);
      else if (strstr(packet, SNMP_COMMUNITY_WRITE) && strstr(packet, "SET=0")) digitalWrite(RELAY1_PIN, LOW);
      sprintf(packet, "%d", digitalRead(RELAY1_PIN));
    } else if (strstr(packet, OID_RELAY2)) {
      if (strstr(packet, SNMP_COMMUNITY_WRITE) && strstr(packet, "SET=1")) digitalWrite(RELAY2_PIN, HIGH);
      else if (strstr(packet, SNMP_COMMUNITY_WRITE) && strstr(packet, "SET=0")) digitalWrite(RELAY2_PIN, LOW);
      sprintf(packet, "%d", digitalRead(RELAY2_PIN));
    }

    // === Uptime (in milliseconds) ===
    else if (strstr(packet, OID_UPTIME)) {
      sprintf(packet, "%lu", millis());
    }

    // === System Info ===
    else if (strstr(packet, OID_NODE_NAME)) {
      strncpy(packet, nodeName.c_str(), sizeof(packet));
    } else if (strstr(packet, OID_BOARD_INFO)) {
      snprintf(packet, sizeof(packet), "ESP32-ETH PoE Node");
    } else if (strstr(packet, OID_CHIP_INFO)) {
      esp_chip_info_t chip_info;
      esp_chip_info(&chip_info);
      snprintf(packet, sizeof(packet), "ESP32 rev%d, %d cores", chip_info.revision, chip_info.cores);
    } else if (strstr(packet, OID_MAC_ADDR)) {
      uint8_t mac[6];
      esp_read_mac(mac, ESP_MAC_ETH);
      snprintf(packet, sizeof(packet), "%02X:%02X:%02X:%02X:%02X:%02X",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else if (strstr(packet, OID_FLASH_SIZE)) {
      sprintf(packet, "%lu", spi_flash_get_chip_size());
    } else if (strstr(packet, OID_SW_VERSION)) {
      strncpy(packet, SOFTWARE_VERSION, sizeof(packet));
    }

    // === Unknown OID ===
    else {
      snprintf(packet, sizeof(packet), "ERR: Unknown OID");
    }

    // === Send SNMP Response ===
    snmpUdp.beginPacket(snmpUdp.remoteIP(), snmpUdp.remotePort());
    snmpUdp.write((const uint8_t *)packet, strlen(packet));
    snmpUdp.endPacket();
  }
}

// === Main Loop ===
void loop() {
  handleSnmp();
  delay(50);
}