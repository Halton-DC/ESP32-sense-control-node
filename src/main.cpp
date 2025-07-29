/**
 * HDC Sense & Control Node - Unified ESP32-S3 + W5500 SNMP Firmware
 * -----------------------------------------------------------------
 * Board: ESP32-S3-ETH (using external W5500 Ethernet over SPI)
 * Sensors:
 *   - Temperature & Humidity: SHT31 (I2C on GPIO 33=SDA, GPIO 34=SCL)
 *   - Airflow: Omron D6F-V03A1 (analog voltage on GPIO 38)
 *   - Contact inputs: dry contacts on GPIO 43 & 44
 *   - Relay outputs: GPIO 36 and 37
 * Networking:
 *   - W5500 Ethernet with static IP
 *   - SNMPv2c agent with read/write support
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <esp_system.h>

#define SOFTWARE_VERSION "1.0.0"

// === SNMP Community Strings ===
#define SNMP_COMMUNITY_READ  "public"
#define SNMP_COMMUNITY_WRITE "private"

// === GPIO Pins ===
#define RELAY1_PIN    36
#define RELAY2_PIN    37
#define CONTACT1_PIN  43
#define CONTACT2_PIN  44
#define AIRFLOW_PIN   38
#define SDA_PIN       33
#define SCL_PIN       34

// === W5500 SPI Pins ===
#define W5500_CS   14
#define W5500_RST  9

// === MAC and IP ===
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 9, 200);
IPAddress gateway(192, 168, 9, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 9, 1);

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
EthernetUDP snmpUdp;
unsigned long bootMillis = 0;
String nodeName = "hdc-node-001";

void setupEthernet() {
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);
  delay(50);
  digitalWrite(W5500_RST, HIGH);
  delay(50);

  SPI.begin(13, 12, 11); // SCK, MISO, MOSI
  Ethernet.init(W5500_CS);
  Ethernet.begin(mac, ip, dns, gateway, subnet);
  delay(1000);

  Serial.print("[ETH] Final IP: ");
  Serial.println(Ethernet.localIP());
}

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

void handleSnmp() {
  char packet[1500];
  int len = snmpUdp.parsePacket();
  if (len > 0) {
    snmpUdp.read(packet, len);

    if (strstr(packet, OID_TEMP_C)) {
      sprintf(packet, "%d", (int)(sht31.readTemperature() * 100));
    } else if (strstr(packet, OID_TEMP_F)) {
      sprintf(packet, "%d", (int)((sht31.readTemperature() * 1.8 + 32) * 100));
    } else if (strstr(packet, OID_HUMIDITY)) {
      sprintf(packet, "%d", (int)(sht31.readHumidity() * 100));
    } else if (strstr(packet, OID_AIRFLOW)) {
      sprintf(packet, "%d", analogRead(AIRFLOW_PIN));
    } else if (strstr(packet, OID_CONTACT1)) {
      sprintf(packet, "%d", digitalRead(CONTACT1_PIN));
    } else if (strstr(packet, OID_CONTACT2)) {
      sprintf(packet, "%d", digitalRead(CONTACT2_PIN));
    } else if (strstr(packet, OID_RELAY1)) {
      if (strstr(packet, SNMP_COMMUNITY_WRITE) && strstr(packet, "SET=1")) digitalWrite(RELAY1_PIN, HIGH);
      else if (strstr(packet, SNMP_COMMUNITY_WRITE) && strstr(packet, "SET=0")) digitalWrite(RELAY1_PIN, LOW);
      sprintf(packet, "%d", digitalRead(RELAY1_PIN));
    } else if (strstr(packet, OID_RELAY2)) {
      if (strstr(packet, SNMP_COMMUNITY_WRITE) && strstr(packet, "SET=1")) digitalWrite(RELAY2_PIN, HIGH);
      else if (strstr(packet, SNMP_COMMUNITY_WRITE) && strstr(packet, "SET=0")) digitalWrite(RELAY2_PIN, LOW);
      sprintf(packet, "%d", digitalRead(RELAY2_PIN));
    } else if (strstr(packet, OID_UPTIME)) {
      sprintf(packet, "%lu", millis());
    } else if (strstr(packet, OID_NODE_NAME)) {
      strncpy(packet, nodeName.c_str(), sizeof(packet));
    } else if (strstr(packet, OID_BOARD_INFO)) {
      snprintf(packet, sizeof(packet), "ESP32-S3 + W5500 Node");
    } else if (strstr(packet, OID_CHIP_INFO)) {
      esp_chip_info_t chip_info;
      esp_chip_info(&chip_info);
      snprintf(packet, sizeof(packet), "ESP32 rev%d, %d cores", chip_info.revision, chip_info.cores);
    } else if (strstr(packet, OID_MAC_ADDR)) {
      uint8_t macAddr[6];
      esp_read_mac(macAddr, ESP_MAC_ETH);
      snprintf(packet, sizeof(packet), "%02X:%02X:%02X:%02X:%02X:%02X",
               macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
    } else if (strstr(packet, OID_FLASH_SIZE)) {
      sprintf(packet, "%lu", spi_flash_get_chip_size());
    } else if (strstr(packet, OID_SW_VERSION)) {
      strncpy(packet, SOFTWARE_VERSION, sizeof(packet));
    } else {
      snprintf(packet, sizeof(packet), "ERR: Unknown OID");
    }

    snmpUdp.beginPacket(snmpUdp.remoteIP(), snmpUdp.remotePort());
    snmpUdp.write((const uint8_t *)packet, strlen(packet));
    snmpUdp.endPacket();
  }
}

void loop() {
  handleSnmp();

  // === DEBUG SENSOR OUTPUT ===
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  int airflow = analogRead(AIRFLOW_PIN);

  Serial.print("Temp: "); Serial.print(t); Serial.print(" °C | ");
  Serial.print("Humidity: "); Serial.print(h); Serial.print(" % | ");
  Serial.print("Airflow: "); Serial.println(airflow);

  delay(1000);
}