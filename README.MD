# HDC Sense & Control Node

Unified firmware for ESP32-based PoE sensor and control devices. This project enables SNMP-based monitoring and control of temperature, humidity, airflow, contact inputs, and relay outputs.

## Features

- 🖧 Ethernet networking (PoE capable)
- 📈 Temperature and humidity via SHT31 (I2C)
- 🌬️ Airflow measurement via Omron D6F-V03A1 (analog)
- 🔌 Two digital contact inputs
- ⚡ Two relay outputs (digital control)
- 📡 SNMPv2c agent with read/write support
- ⏱️ System uptime reporting
- 🌐 Static IP with DHCP fallback
- 🧠 Node info query: board, chip ID, revision, MAC, etc.
- 🔐 Configurable SNMP read/write communities

## Hardware Requirements

- ESP32 board with Ethernet and PoE (e.g., Olimex ESP32-POE, ESP32-S3-ETH)
- Sensors:
  - SHT31 (I2C temperature/humidity)
  - Omron D6F-V03A1 (analog airflow)
- Optional:
  - Two dry contact inputs (GPIO 32/33)
  - Two relay outputs (GPIO 25/26)

## Wiring Summary

| Signal        | GPIO | Notes                     |
|---------------|------|---------------------------|
| Relay 1       | 25   | Output                    |
| Relay 2       | 26   | Output                    |
| Contact 1     | 32   | Input with pull-up        |
| Contact 2     | 33   | Input with pull-up        |
| Airflow       | 34   | Analog input              |
| I2C SDA       | 21   | Connect to SHT31          |
| I2C SCL       | 22   | Connect to SHT31          |

## PlatformIO Setup

```ini
[env:esp32dev]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
upload_speed = 921600

lib_deps =
  adafruit/Adafruit SHT31 Library@^2.2.0
  adafruit/Adafruit BusIO@^1.14.2

build_flags =
  -D ETH_CLK_MODE=ETH_CLOCK_GPIO17_OUT
  -D ETH_PHY_POWER=12
  -D ETH_TYPE=ETH_PHY_LAN8720
  -D ETH_ADDR=1
  -D ETH_MDC=23
  -D ETH_MDIO=18
```

## Build & Upload

1. Clone the repository:
    ```bash
    git clone https://github.com/hdc/hdc-sense-control-node.git
    cd hdc-sense-control-node
    ```

2. Open in [Visual Studio Code](https://code.visualstudio.com) with [PlatformIO extension](https://platformio.org/install/ide?install=vscode)

3. Connect your ESP32 via USB.

4. Upload firmware:
    ```bash
    pio run --target upload
    ```

## SNMP Configuration

- SNMPv2c support with GET and SET
- Default community strings are defined in `main.cpp`:
  ```cpp
  const char* SNMP_COMMUNITY_READ  = "public";
  const char* SNMP_COMMUNITY_WRITE = "private";
  ```

### OID Summary

| Metric             | OID                       | Access    |
|--------------------|---------------------------|-----------|
| Temperature (°C)    | `.1.3.6.1.4.1.55555.1.1.0` | Read-only |
| Temperature (°F)    | `.1.3.6.1.4.1.55555.1.11.0`| Read-only |
| Humidity (%)        | `.1.3.6.1.4.1.55555.1.2.0` | Read-only |
| Airflow (V × 1000)  | `.1.3.6.1.4.1.55555.1.3.0` | Read-only |
| Contact 1 state     | `.1.3.6.1.4.1.55555.1.6.0` | Read-only |
| Contact 2 state     | `.1.3.6.1.4.1.55555.1.7.0` | Read-only |
| Relay 1 state       | `.1.3.6.1.4.1.55555.1.4.0` | RW        |
| Relay 2 state       | `.1.3.6.1.4.1.55555.1.5.0` | RW        |
| Uptime (sec)        | `.1.3.6.1.4.1.55555.1.99.0`| Read-only |
| Board Info          | `.1.3.6.1.4.1.55555.1.20.0`| Read-only |
| Chip Model          | `.1.3.6.1.4.1.55555.1.21.0`| Read-only |
| Chip Cores          | `.1.3.6.1.4.1.55555.1.22.0`| Read-only |
| Chip Revision       | `.1.3.6.1.4.1.55555.1.23.0`| Read-only |
| Flash Size (bytes)  | `.1.3.6.1.4.1.55555.1.24.0`| Read-only |
| MAC Address         | `.1.3.6.1.4.1.55555.1.25.0`| Read-only |
| Software Version    | `.1.3.6.1.4.1.55555.1.30.0`| Read-only |
| Node Name           | `.1.3.6.1.4.1.55555.1.31.0`| RW        |

## MIB File

A full MIB file is included in `hdc-sense-control.mib`. You can load it in tools like:

- [SNMPB](https://sourceforge.net/projects/snmpb/)
- Zabbix SNMP template import
- LibreNMS or Observium

## Zabbix Integration

A Zabbix YAML template is available in `zabbix-hdc-sense.yaml`. It includes:

- Host macros for SNMP community
- Items for all sensors and relays
- Triggers for offline/missing data
- Graphs for temperature, humidity, airflow

## License

MIT License

## Contact

Halton Datacenter  
Antoine Boucher  
https://haltondc.com  
