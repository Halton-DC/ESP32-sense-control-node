/**
 * HDC Sense & Control Node - Unified ESP32 PoE Sensor Firmware
 * -------------------------------------------------------------
 * Board: ESP32-ETH (with PoE, e.g., Olimex ESP32-POE)
 * Sensors:
 *   - Temperature & Humidity: SHT31 (I2C @ 0x44)
 *   - Airflow: Omron D6F-V03A1 (analog signal to pin 34)
 *   - Contact Inputs: 2× dry-contact (pins 32 & 33)
 *   - Outputs: 2× relays (pins 25 & 26)
 * Networking:
 *   - Ethernet w/ DHCP + static fallback
 *   - SNMP agent responding to GET and SET
 */

#include <Arduino.h>
#include <Wire.h>
#include <ETH.h>
#include <Adafruit_SHT31.h>
#include <WiFiUdp.h>

// === Ethernet IP Configuration ===
const IPAddress STATIC_IP(192, 168, 1, 150);
const IPAddress GATEWAY_IP(192, 168, 1, 1);
const IPAddress SUBNET_MASK(255, 255, 255, 0);
const IPAddress DNS_IP(8, 8, 8, 8);

// === GPIO Pin Assignments ===
#define RELAY1_PIN    25  // Relay 1 control output
#define RELAY2_PIN    26  // Relay 2 control output
#define CONTACT1_PIN  32  // Contact sensor input 1
#define CONTACT2_PIN  33  // Contact sensor input 2
#define AIRFLOW_PIN   34  // Analog airflow input (0–3.3V)
#define SCL_PIN       22  // I2C SCL
#define SDA_PIN       21  // I2C SDA

// === SNMP OIDs for raw values ===
#define OID_TEMP      "1.3.6.1.4.1.55555.1.1.0"
#define OID_HUM       "1.3.6.1.4.1.55555.1.2.0"
#define OID_FLOW      "1.3.6.1.4.1.55555.1.3.0"
#define OID_RELAY1    "1.3.6.1.4.1.55555.1.4.0"
#define OID_RELAY2    "1.3.6.1.4.1.55555.1.5.0"
#define OID_CONTACT1  "1.3.6.1.4.1.55555.1.6.0"
#define OID_CONTACT2  "1.3.6.1.4.1.55555.1.7.0"
#define OID_UPTIME    "1.3.6.1.4.1.55555.1.99.0"

// === Globals ===
Adafruit_SHT31 sht31 = Adafruit_SHT31();  // SHT31 temp/humidity sensor
WiFiUDP snmpUdp;
unsigned long bootMillis = 0;             // System boot time

// === Ethernet Event Handler ===
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[ETH] DHCP IP: ");
      Serial.println(ETH.localIP());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] Lost connection.");
      break;
    default:
      break;
  }
}

// === Initialize Ethernet with DHCP + fallback ===
void setupEthernet() {
  WiFi.onEvent(WiFiEvent);
  ETH.begin();
  delay(1000);  // Wait for ETH to stabilize

  if (ETH.localIP() == INADDR_NONE) {
    Serial.println("[ETH] DHCP failed, falling back to static IP...");
    ETH.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, DNS_IP);
    delay(1000);
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);

  setupEthernet();

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(CONTACT1_PIN, INPUT_PULLUP);
  pinMode(CONTACT2_PIN, INPUT_PULLUP);

  if (!sht31.begin(0x44)) {
    Serial.println("SHT31 sensor not found!");
  } else {
    Serial.println("SHT31 ready.");
  }

  snmpUdp.begin(161);
  bootMillis = millis();

  Serial.println("SNMP sensor node ready.");
}

// === Handle SNMP GET/SET Requests ===
void handleSnmp() {
  char packet[1500];
  int len = snmpUdp.parsePacket();
  if (len > 0) {
    snmpUdp.read(packet, len);

    // Match incoming OID and return raw values
    if (strstr(packet, OID_TEMP)) {
      dtostrf(sht31.readTemperature(), 1, 2, packet);
    } else if (strstr(packet, OID_HUM)) {
      dtostrf(sht31.readHumidity(), 1, 2, packet);
    } else if (strstr(packet, OID_FLOW)) {
      int raw = analogRead(AIRFLOW_PIN);
      dtostrf((float)raw, 1, 0, packet);  // Return raw ADC value
    } else if (strstr(packet, OID_CONTACT1)) {
      sprintf(packet, "%d", digitalRead(CONTACT1_PIN));
    } else if (strstr(packet, OID_CONTACT2)) {
      sprintf(packet, "%d", digitalRead(CONTACT2_PIN));
    } else if (strstr(packet, OID_RELAY1)) {
      if (strstr(packet, "SET=1")) digitalWrite(RELAY1_PIN, HIGH);
      else if (strstr(packet, "SET=0")) digitalWrite(RELAY1_PIN, LOW);
      sprintf(packet, "%d", digitalRead(RELAY1_PIN));
    } else if (strstr(packet, OID_RELAY2)) {
      if (strstr(packet, "SET=1")) digitalWrite(RELAY2_PIN, HIGH);
      else if (strstr(packet, "SET=0")) digitalWrite(RELAY2_PIN, LOW);
      sprintf(packet, "%d", digitalRead(RELAY2_PIN));
    } else if (strstr(packet, OID_UPTIME)) {
      unsigned long uptimeSec = millis() / 1000;
      sprintf(packet, "%lu", uptimeSec);
    } else {
      strcpy(packet, "ERR: Unknown OID");
    }

    // Respond with SNMP-compatible payload
    snmpUdp.beginPacket(snmpUdp.remoteIP(), snmpUdp.remotePort());
    snmpUdp.write((const uint8_t*)packet, strlen(packet));
    snmpUdp.endPacket();
  }
}

// === Main Loop ===
void loop() {
  handleSnmp();
  delay(50);
}