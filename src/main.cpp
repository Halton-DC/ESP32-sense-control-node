#include <Arduino.h>
#include <Wire.h>
#include <ETH.h>
#include <Adafruit_SHT31.h>
#include <WiFiUdp.h>

// === Pins ===
#define RELAY1_PIN 25
#define RELAY2_PIN 26
#define CONTACT1_PIN 32
#define CONTACT2_PIN 33
#define AIRFLOW_PIN 34  // Analog airflow signal from D6F-V03A1

// === SNMP OIDs ===
#define OID_TEMP "1.3.6.1.4.1.55555.1.1.0"
#define OID_HUM  "1.3.6.1.4.1.55555.1.2.0"
#define OID_FLOW "1.3.6.1.4.1.55555.1.3.0"
#define OID_RELAY1 "1.3.6.1.4.1.55555.1.4.0"
#define OID_RELAY2 "1.3.6.1.4.1.55555.1.5.0"
#define OID_CONTACT1 "1.3.6.1.4.1.55555.1.6.0"
#define OID_CONTACT2 "1.3.6.1.4.1.55555.1.7.0"

// === Global ===
Adafruit_SHT31 sht31 = Adafruit_SHT31();
WiFiUDP snmpUdp;

// === Ethernet Events ===
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[ETH] IP Address: ");
      Serial.println(ETH.localIP());
      break;
    default: break;
  }
}

void setupEthernet() {
  WiFi.onEvent(WiFiEvent);
  ETH.begin();
}

// === Initialization ===
void setup() {
  Serial.begin(115200);
  delay(1000);

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

  snmpUdp.begin(161);  // SNMP agent listens on port 161

  Serial.println("System ready.");
}

// === Simple SNMP GET/SET Handler ===
void handleSnmp() {
  char packet[1500];
  int len = snmpUdp.parsePacket();
  if (len > 0) {
    snmpUdp.read(packet, len);

    // Basic OID handler logic (stubbed for demo)
    if (strstr(packet, OID_TEMP)) {
      float temp = sht31.readTemperature();
      snprintf(packet, sizeof(packet), "Temp: %.2f", temp);
    } else if (strstr(packet, OID_HUM)) {
      float hum = sht31.readHumidity();
      snprintf(packet, sizeof(packet), "Humidity: %.2f", hum);
    } else if (strstr(packet, OID_FLOW)) {
      int raw = analogRead(AIRFLOW_PIN);
      float voltage = raw * 3.3 / 4095;
      snprintf(packet, sizeof(packet), "Airflow V: %.2f", voltage);
    } else if (strstr(packet, OID_CONTACT1)) {
      int state = digitalRead(CONTACT1_PIN);
      snprintf(packet, sizeof(packet), "Contact1: %d", state);
    } else if (strstr(packet, OID_CONTACT2)) {
      int state = digitalRead(CONTACT2_PIN);
      snprintf(packet, sizeof(packet), "Contact2: %d", state);
    } else if (strstr(packet, OID_RELAY1)) {
      if (strstr(packet, "SET=1")) digitalWrite(RELAY1_PIN, HIGH);
      else if (strstr(packet, "SET=0")) digitalWrite(RELAY1_PIN, LOW);
      snprintf(packet, sizeof(packet), "Relay1: %d", digitalRead(RELAY1_PIN));
    } else if (strstr(packet, OID_RELAY2)) {
      if (strstr(packet, "SET=1")) digitalWrite(RELAY2_PIN, HIGH);
      else if (strstr(packet, "SET=0")) digitalWrite(RELAY2_PIN, LOW);
      snprintf(packet, sizeof(packet), "Relay2: %d", digitalRead(RELAY2_PIN));
    } else {
      snprintf(packet, sizeof(packet), "Unknown OID");
    }

    snmpUdp.beginPacket(snmpUdp.remoteIP(), snmpUdp.remotePort());
    snmpUdp.write((const uint8_t *)packet, strlen(packet));
    snmpUdp.endPacket();
  }
}

void loop() {
  handleSnmp();
  delay(50);
}