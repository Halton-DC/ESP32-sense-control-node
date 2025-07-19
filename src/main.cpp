#include <Arduino.h>
#include <Wire.h>
#include <ETH.h>
#include <SNMP_Agent.h>
#include <Adafruit_SHT31.h>

#define AIRFLOW_PIN 4
#define CONTACT1_PIN 18
#define CONTACT2_PIN 19

Adafruit_SHT31 sht31 = Adafruit_SHT31();
SNMPAgent snmp;

float readAirflow() {
  int raw = analogRead(AIRFLOW_PIN);
  float voltage = raw * 3.3 / 4095.0;
  float mps = (voltage - 0.5) * (3.0 / 2.0); // D6F-V03A1: 0.5–2.5V for 0–3m/s
  if (mps < 0) mps = 0;
  return mps;
}

int readContact1() {
  return digitalRead(CONTACT1_PIN) == LOW ? 1 : 0;
}

int readContact2() {
  return digitalRead(CONTACT2_PIN) == LOW ? 1 : 0;
}

void WiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_ETH_GOT_IP) {
    Serial.print("IP address: ");
    Serial.println(ETH.localIP());
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // SDA, SCL

  pinMode(AIRFLOW_PIN, INPUT);
  pinMode(CONTACT1_PIN, INPUT_PULLUP);
  pinMode(CONTACT2_PIN, INPUT_PULLUP);

  if (!sht31.begin(0x44)) {
    Serial.println("SHT31 not found");
  }

  WiFi.onEvent(WiFiEvent);
  ETH.begin();

  snmp.begin("public");

  snmp.addScalarOID("1.3.6.1.4.1.53864.1.1.0", SNMP_TYPE_INTEGER, []() {
    return (int)(readAirflow() * 100);
  });

  snmp.addScalarOID("1.3.6.1.4.1.53864.1.2.0", SNMP_TYPE_INTEGER, []() {
    return (int)(sht31.readTemperature() * 100);
  });

  snmp.addScalarOID("1.3.6.1.4.1.53864.1.3.0", SNMP_TYPE_INTEGER, []() {
    return readContact1();
  });

  snmp.addScalarOID("1.3.6.1.4.1.53864.1.4.0", SNMP_TYPE_INTEGER, []() {
    return readContact2();
  });

  snmp.addScalarOID("1.3.6.1.4.1.53864.1.5.0", SNMP_TYPE_INTEGER, []() {
    return millis() / 1000;
  });

  Serial.println("SNMP agent ready.");
}

void loop() {
  snmp.handle();
  delay(100);
}