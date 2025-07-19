# HDC Sense & Control Node Firmware

**Unified ESP32 PoE Sensor Firmware with SNMP for Zabbix and Datacenter Monitoring**

---

## 📌 Overview

This firmware powers **ESP32-ETH** based sensor nodes designed for rack-level or room-level environment monitoring and control in data centers or industrial settings. It supports **SNMPv2c GET/SET**, **Ethernet with fallback networking**, and sensors such as:

- **Temperature & Humidity** – SHT31 via I2C
- **Airflow** – Omron D6F-V03A1 (analog input)
- **Dry contact inputs** – door sensors, alarms, etc.
- **Relay outputs** – for control (e.g., fan or buzzer)
- **Hardware Metadata** – chip info, MAC, flash size
- **SNMP agent** – built-in, fully integrated

---

## 🌐 Networking Configuration

The ESP32-ETH node uses **static IP by default**, with automatic fallback to **DHCP** if no static configuration is defined or fails.

### Static IP (default)
Configured at the top of `main.cpp` using constants:

```cpp
const IPAddress STATIC_IP(192, 168, 1, 150);
const IPAddress GATEWAY_IP(192, 168, 1, 1);
const IPAddress SUBNET_MASK(255, 255, 255, 0);
const IPAddress DNS_IP(8, 8, 8, 8);
```

You can modify these to suit your network. The device will attempt to use this IP first.

### DHCP Fallback
If the static IP assignment fails (e.g., network unavailable or misconfigured), the firmware falls back to DHCP automatically.

You’ll see status messages over serial:
- `[ETH] Static IP assigned: ...`
- `[ETH] Static IP failed, requesting DHCP...`

Ethernet configuration uses the **LAN8720 PHY** with GPIO-based clock and power control.

### Ethernet Wiring
- MDC: GPIO23
- MDIO: GPIO18
- Power: GPIO12
- Clock Out: GPIO17 (ETH_CLOCK_GPIO17_OUT)

Ensure your board matches these pin assignments or adjust `platformio.ini` and `main.cpp` accordingly.

---

## 🧪 OID Reference

| Metric                  | OID                          | Type        | Format     |
|-------------------------|------------------------------|-------------|------------|
| Temperature (°C x100)   | `.1.3.6.1.4.1.55555.1.1.0`    | Read        | Integer    |
| Temperature (°F x100)   | `.1.3.6.1.4.1.55555.1.2.0`    | Read        | Integer    |
| Humidity (%RH x100)     | `.1.3.6.1.4.1.55555.1.3.0`    | Read        | Integer    |
| Airflow (ADC 0–4095)    | `.1.3.6.1.4.1.55555.1.4.0`    | Read        | Integer    |
| Relay 1 State           | `.1.3.6.1.4.1.55555.1.5.0`    | Read/Write  | 0 / 1      |
| Relay 2 State           | `.1.3.6.1.4.1.55555.1.6.0`    | Read/Write  | 0 / 1      |
| Contact 1               | `.1.3.6.1.4.1.55555.1.7.0`    | Read        | 0 / 1      |
| Contact 2               | `.1.3.6.1.4.1.55555.1.8.0`    | Read        | 0 / 1      |
| Uptime (ms)             | `.1.3.6.1.4.1.55555.1.9.0`    | Read        | Integer    |
| Node Name               | `.1.3.6.1.4.1.55555.1.10.0`   | Read        | String     |
| Board Info              | `.1.3.6.1.4.1.55555.1.11.0`   | Read        | String     |
| Chip Info               | `.1.3.6.1.4.1.55555.1.12.0`   | Read        | String     |
| MAC Address             | `.1.3.6.1.4.1.55555.1.13.0`   | Read        | String     |
| Flash Size (bytes)      | `.1.3.6.1.4.1.55555.1.14.0`   | Read        | Integer    |
| Software Version        | `.1.3.6.1.4.1.55555.1.15.0`   | Read        | String     |

---

## 🛠️ Setup

Instructions moved here...

---

