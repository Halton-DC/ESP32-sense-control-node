# HDC Sense & Control Node  !!! WORK IN PROGRESS

**Data-center environmental monitoring node** on the ESP32-S3-ETH (W5500).
Deployed across the data center to report **temperature, humidity and airflow**
to Zabbix over SNMP, with a secure, self-hosted management interface.

- **Monitoring** — SNMPv2c (private community) → Zabbix
- **Management** — HTTPS web UI + authenticated REST API (TLS, on-device self-signed cert)
- **Connectivity** — Ethernet **and** Wi-Fi (WPA2/WPA3), DHCP with static fallback,
  automatic **SoftAP provisioning fallback** so the node is always reachable
- **Security** — forced first-login setup (change username + password), PBKDF2-hashed
  credentials, session cookies, per-device TLS certificate
- **Serviceable** — persistent config in NVS, factory reset (button or REST),
  downloadable MIB + Zabbix 7.0 template straight from the web UI

> Status: firmware validated end-to-end on hardware — Wi-Fi (WPA3) join, HTTPS UI,
> REST auth + relay control, SNMP GET/WALK/SET, forced setup, and downloads.

## Hardware

| Component          | Description                                    |
|--------------------|------------------------------------------------|
| **Board**          | ESP32-S3-ETH                                   |
| **Ethernet**       | W5500 (SPI) — exposed as an lwIP `ETH` interface |
| **Temp/Humidity**  | SHT31 (I²C `0x44`)                             |
| **Airflow**        | Omron D6F-V03A1 → GPIO 1 (ADC)                 |
| **Contact inputs** | GPIO 43, 44 (dry contact, `INPUT_PULLUP`)      |
| **Relays**         | GPIO 36, 37                                    |
| **I²C**            | SDA 33, SCL 34                                 |
| **W5500 SPI**      | CS 14, RST 9, INT 10, MISO 12, MOSI 11, SCK 13 |
| **Factory reset**  | BOOT button (GPIO 0), hold 5 s at power-up     |

> `Serial` is routed to the built-in USB-Serial-JTAG (`ARDUINO_USB_MODE=1`,
> `ARDUINO_USB_CDC_ON_BOOT=1`), which frees UART0's default pins (GPIO 43/44) for
> the contact inputs.

## Toolchain

Requires **arduino-esp32 3.x** (via the pioarduino platform) so the W5500 is a
first-class lwIP interface that TLS/HTTPS can run over.

```ini
[env:esp32-s3-eth]
platform  = https://github.com/pioarduino/platform-espressif32/releases/download/54.03.21/platform-espressif32.zip
board     = esp32-s3-devkitc-1
framework = arduino

lib_deps =
  adafruit/Adafruit SHT31 Library@^2.2.0
  adafruit/Adafruit BusIO@^1.14.2
  bblanchon/ArduinoJson@^7.0.0
  https://github.com/shortbloke/Arduino_SNMP.git
```

```bash
pio run                 # build
pio run -t upload       # flash
pio device monitor      # serial
```

> If a build fails at `bootloader.bin` with `get_metavar() missing … 'ctx'`,
> pin PlatformIO's Python `click` to 8.1.x: `~/.platformio/penv/bin/pip install "click==8.1.8"`.

## First-boot setup

1. Power the node. It joins Ethernet/Wi-Fi if configured; otherwise it raises a
   WPA2 setup AP named `HDC-Node-XXXX` (default passphrase `hdc-setup`).
2. Browse to `https://<node-ip>/` (accept the self-signed certificate).
3. Sign in with the **initial admin password printed to the serial console**
   on first boot (production builds generate a unique random one per device).
4. You are **required** to set a new **username and password** before anything
   else is accessible.

## Networking

- **Ethernet + Wi-Fi run together** — the UI and SNMP answer on whichever
  interface has an address.
- **DHCP first, static fallback** — configurable in the UI.
- **DHCP hostname** — preset from the node name, editable, so the node shows by
  name in the router's lease list.
- **SoftAP fallback** — if neither wired nor Wi-Fi connects, the node hosts its
  own AP + web UI for provisioning.

## SNMP

Enterprise tree, base `.1.3.6.1.4.1.55555.1` (see `HDC-SENSE-CONTROL-MIB.txt`):

| OID (…55555.1.x)  | Access | Description                    |
|-------------------|--------|--------------------------------|
| `.1.0`  | RO | Temperature (°C ×100)               |
| `.2.0`  | RO | Temperature (°F ×100)               |
| `.3.0`  | RO | Humidity (%RH ×100)                 |
| `.4.0`  | RO | Airflow (raw ADC 0–4095)            |
| `.5.0`  | RW | Relay 1 (0/1)                       |
| `.6.0`  | RW | Relay 2 (0/1)                       |
| `.7.0`  | RO | Contact 1 (0/1)                    |
| `.8.0`  | RO | Contact 2 (0/1)                    |
| `.9.0`  | RO | Uptime (TimeTicks)                 |
| `.10.0` | RO | Node name                          |
| `.11.0` | RO | Board info                         |
| `.12.0` | RO | Chip info                          |
| `.13.0` | RO | MAC address                        |
| `.14.0` | RO | Flash size (bytes)                 |
| `.15.0` | RO | Software version                   |

Also serves the **MIB-II system group** (`sysDescr`, `sysUpTime`, `sysContact`,
`sysName`, `sysLocation`) for generic walks and Zabbix discovery.

Communities are **private by default** (`hdc-ro-…` / `hdc-rw-…`) and editable in
the UI. SNMPv2c is unencrypted — keep it on a trusted management VLAN; for secure
control use the HTTPS REST API. Relay writes over SNMP require the write community
and can be disabled.

```bash
snmpget  -v2c -c <read-community>  <ip> 1.3.6.1.4.1.55555.1.1.0     # temperature
snmpwalk -v2c -c <read-community>  <ip> 1.3.6.1.4.1.55555           # full tree
snmpset  -v2c -c <write-community> <ip> 1.3.6.1.4.1.55555.1.5.0 i 1 # relay 1 on
```

## REST API (HTTPS, session-cookie auth)

| Method & path            | Purpose                                    |
|--------------------------|--------------------------------------------|
| `POST /api/login`        | Authenticate, receive session cookie       |
| `POST /api/logout`       | Invalidate session                         |
| `GET  /api/status`       | Live telemetry + network state             |
| `GET  /api/settings`     | Current configuration (no secrets)         |
| `POST /api/settings`     | Update identity/network/Wi-Fi/SNMP/relays  |
| `POST /api/relay`        | Set a relay (`{"relay":1,"state":1}`)      |
| `POST /api/password`     | Change username + password                 |
| `POST /api/reboot`       | Reboot                                     |
| `POST /api/factory-reset`| Erase all config + TLS cert, reboot        |
| `GET  /api/mib`          | Download the SNMP MIB                       |
| `GET  /api/zabbix`       | Download the Zabbix 7.0 template           |
| `GET  /api/grafana`      | Download the Grafana dashboard (JSON)      |
| `GET  /metrics`          | Prometheus exposition (optional bearer)    |

Until the factory password is changed, all endpoints except login/logout/status
and password return **403 `password_change_required`**.

## Zabbix

`zabbix-hdc-sense.yaml` is a **Zabbix 7.0+** template (also downloadable from the
node's **System → Monitoring integration**, with the read community pre-filled).
Import via **Data collection → Templates → Import**, link it to the node's host,
and set the SNMPv2 interface community to `{$SNMP_COMMUNITY}`. Includes items for
all metrics, value maps for relay/contact states, and temperature/humidity triggers.

## Prometheus & Grafana

The node serves a Prometheus exposition endpoint at **`/metrics`** (`hdc_*`
gauges + an `hdc_info` label set). It is open by default (for a trusted VLAN);
set a **bearer token** in the UI (System → Prometheus & Grafana) to require auth.
A ready **Grafana dashboard** is downloadable from the same page.

```yaml
# prometheus.yml
scrape_configs:
  - job_name: hdc-nodes
    scheme: https
    tls_config: { insecure_skip_verify: true }   # self-signed cert
    # authorization: { credentials: "<bearer-token>" }   # if a token is set
    static_configs:
      - targets: ['<node-ip>:443']
```

## Fleet deployment

The **MAC is derived from the chip ID** (unique per unit, no config). Identity and
location are set in the UI, or baked per build:

```ini
[env:rack-a01]
extends = env:esp32-s3-eth
build_flags =
  ${env:esp32-s3-eth.build_flags}
  -DDEFAULT_NODE_NAME='"hdc-node-a01"'
  -DDEFAULT_NODE_LOCATION='"DC1 / Row A / Rack 01"'
```

> **Production builds must remove the DEV-only build flags** in `platformio.ini`
> (default Wi-Fi credentials and the fixed `DEFAULT_ADMIN_PASSWORD`). Without the
> fixed admin password, each unit generates a unique random one at first boot.

## License

MIT License · © 2025 Halton Datacenter — Antoine Boucher.
