/**
 * integrations.h - Downloadable monitoring assets served by the web UI.
 *   - MIB_TXT     : the SNMP MIB (enterprise .1.3.6.1.4.1.55555)
 *   - ZABBIX_YAML : a Zabbix 7.0+ template (import via Data collection > Templates)
 * The token __SNMP_COMMUNITY__ in ZABBIX_YAML is replaced at serve time with the
 * node's configured read community.
 */
#pragma once
#include <Arduino.h>

static const char MIB_TXT[] PROGMEM = R"MIB(HDC-SENSE-CONTROL-MIB DEFINITIONS ::= BEGIN

IMPORTS
  MODULE-IDENTITY, OBJECT-TYPE, enterprises, Integer32, TimeTicks, DisplayString
    FROM SNMPv2-SMI;

hdcSenseControl MODULE-IDENTITY
  LAST-UPDATED "202607080000Z"
  ORGANIZATION "Halton Datacenter"
  CONTACT-INFO "Antoine Boucher, Halton Datacenter"
  DESCRIPTION  "SNMP MIB for the HDC Sense & Control Node (ESP32-S3)."
  ::= { enterprises 55555 }

hdcDevice OBJECT IDENTIFIER ::= { hdcSenseControl 1 }

hdcTemperatureC   OBJECT-TYPE SYNTAX Integer32   MAX-ACCESS read-only  STATUS current
  DESCRIPTION "Temperature in Celsius x100 (2573 = 25.73 C)"      ::= { hdcDevice 1 }
hdcTemperatureF   OBJECT-TYPE SYNTAX Integer32   MAX-ACCESS read-only  STATUS current
  DESCRIPTION "Temperature in Fahrenheit x100"                    ::= { hdcDevice 2 }
hdcHumidity       OBJECT-TYPE SYNTAX Integer32   MAX-ACCESS read-only  STATUS current
  DESCRIPTION "Relative humidity percent x100"                    ::= { hdcDevice 3 }
hdcAirflow        OBJECT-TYPE SYNTAX Integer32   MAX-ACCESS read-only  STATUS current
  DESCRIPTION "Air velocity from the D6F-V03A1, m/s x100 (234 = 2.34 m/s)" ::= { hdcDevice 4 }
hdcRelay1         OBJECT-TYPE SYNTAX Integer32   MAX-ACCESS read-write STATUS current
  DESCRIPTION "Relay 1 output state (0=off, 1=on)"               ::= { hdcDevice 5 }
hdcRelay2         OBJECT-TYPE SYNTAX Integer32   MAX-ACCESS read-write STATUS current
  DESCRIPTION "Relay 2 output state (0=off, 1=on)"               ::= { hdcDevice 6 }
hdcContact1       OBJECT-TYPE SYNTAX Integer32   MAX-ACCESS read-only  STATUS current
  DESCRIPTION "Contact input 1 (0=open, 1=closed)"              ::= { hdcDevice 7 }
hdcContact2       OBJECT-TYPE SYNTAX Integer32   MAX-ACCESS read-only  STATUS current
  DESCRIPTION "Contact input 2 (0=open, 1=closed)"              ::= { hdcDevice 8 }
hdcUptime         OBJECT-TYPE SYNTAX TimeTicks   MAX-ACCESS read-only  STATUS current
  DESCRIPTION "System uptime in hundredths of a second"          ::= { hdcDevice 9 }
hdcNodeName       OBJECT-TYPE SYNTAX DisplayString MAX-ACCESS read-only STATUS current
  DESCRIPTION "Device / node identifier"                         ::= { hdcDevice 10 }
hdcBoardInfo      OBJECT-TYPE SYNTAX DisplayString MAX-ACCESS read-only STATUS current
  DESCRIPTION "Board info string"                                ::= { hdcDevice 11 }
hdcChipInfo       OBJECT-TYPE SYNTAX DisplayString MAX-ACCESS read-only STATUS current
  DESCRIPTION "ESP32 chip model / revision / cores"              ::= { hdcDevice 12 }
hdcMacAddress     OBJECT-TYPE SYNTAX DisplayString MAX-ACCESS read-only STATUS current
  DESCRIPTION "Primary MAC address"                              ::= { hdcDevice 13 }
hdcFlashSize      OBJECT-TYPE SYNTAX Integer32   MAX-ACCESS read-only  STATUS current
  DESCRIPTION "Flash chip size in bytes"                         ::= { hdcDevice 14 }
hdcSoftwareVersion OBJECT-TYPE SYNTAX DisplayString MAX-ACCESS read-only STATUS current
  DESCRIPTION "Firmware version string"                          ::= { hdcDevice 15 }

END
)MIB";

static const char ZABBIX_YAML[] PROGMEM = R"ZBX(zabbix_export:
  version: '7.0'
  template_groups:
    - uuid: 0a000001b2b2b2b2c3c3c3c3d4d4d4d4
      name: Templates/Datacenter
  value_maps: []
  templates:
    - uuid: 0a000002b2b2b2b2c3c3c3c3d4d4d4d4
      template: 'HDC Sense Control Node'
      name: 'HDC Sense & Control Node'
      description: 'SNMPv2c monitoring for the HDC Sense & Control Node (ESP32-S3). Enterprise OID base .1.3.6.1.4.1.55555. Set the host SNMP interface community to {$SNMP_COMMUNITY}.'
      groups:
        - name: Templates/Datacenter
      macros:
        - macro: '{$SNMP_COMMUNITY}'
          value: '__SNMP_COMMUNITY__'
          description: 'SNMP v2c read community of the node'
        - macro: '{$HDC.TEMP.HIGH}'
          value: '30'
        - macro: '{$HDC.TEMP.CRIT}'
          value: '35'
        - macro: '{$HDC.HUM.HIGH}'
          value: '70'
      valuemaps:
        - uuid: 0a000003b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'HDC Relay State'
          mappings:
            - {value: '0', newvalue: 'Off'}
            - {value: '1', newvalue: 'On'}
        - uuid: 0a000004b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'HDC Contact State'
          mappings:
            - {value: '0', newvalue: 'Open'}
            - {value: '1', newvalue: 'Closed'}
      items:
        - uuid: 0a000101b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Temperature'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.1.0'
          key: hdc.temp.c
          delay: 30s
          value_type: FLOAT
          units: '!°C'
          preprocessing:
            - type: MULTIPLIER
              parameters: ['0.01']
          tags:
            - {tag: component, value: environment}
          triggers:
            - uuid: 0a000201b2b2b2b2c3c3c3c3d4d4d4d4
              expression: 'last(/HDC Sense Control Node/hdc.temp.c)>{$HDC.TEMP.HIGH}'
              name: 'High temperature ({ITEM.LASTVALUE1})'
              priority: WARNING
            - uuid: 0a000202b2b2b2b2c3c3c3c3d4d4d4d4
              expression: 'last(/HDC Sense Control Node/hdc.temp.c)>{$HDC.TEMP.CRIT}'
              name: 'Critical temperature ({ITEM.LASTVALUE1})'
              priority: HIGH
        - uuid: 0a000102b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Temperature (F)'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.2.0'
          key: hdc.temp.f
          delay: 30s
          value_type: FLOAT
          units: '!°F'
          preprocessing:
            - type: MULTIPLIER
              parameters: ['0.01']
          tags:
            - {tag: component, value: environment}
        - uuid: 0a000103b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Humidity'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.3.0'
          key: hdc.humidity
          delay: 30s
          value_type: FLOAT
          units: '%'
          preprocessing:
            - type: MULTIPLIER
              parameters: ['0.01']
          tags:
            - {tag: component, value: environment}
          triggers:
            - uuid: 0a000203b2b2b2b2c3c3c3c3d4d4d4d4
              expression: 'last(/HDC Sense Control Node/hdc.humidity)>{$HDC.HUM.HIGH}'
              name: 'High humidity ({ITEM.LASTVALUE1})'
              priority: WARNING
        - uuid: 0a000104b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Air velocity'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.4.0'
          key: hdc.airflow
          delay: 30s
          value_type: FLOAT
          units: 'm/s'
          preprocessing:
            - type: MULTIPLIER
              parameters: ['0.01']
          tags:
            - {tag: component, value: environment}
        - uuid: 0a000105b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Relay 1'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.5.0'
          key: hdc.relay1
          delay: 30s
          value_type: UNSIGNED
          valuemap: {name: 'HDC Relay State'}
          tags:
            - {tag: component, value: control}
        - uuid: 0a000106b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Relay 2'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.6.0'
          key: hdc.relay2
          delay: 30s
          value_type: UNSIGNED
          valuemap: {name: 'HDC Relay State'}
          tags:
            - {tag: component, value: control}
        - uuid: 0a000107b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Contact 1'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.7.0'
          key: hdc.contact1
          delay: 30s
          value_type: UNSIGNED
          valuemap: {name: 'HDC Contact State'}
          tags:
            - {tag: component, value: input}
        - uuid: 0a000108b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Contact 2'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.8.0'
          key: hdc.contact2
          delay: 30s
          value_type: UNSIGNED
          valuemap: {name: 'HDC Contact State'}
          tags:
            - {tag: component, value: input}
        - uuid: 0a000109b2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Uptime'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.9.0'
          key: hdc.uptime
          delay: 60s
          value_type: FLOAT
          units: uptime
          preprocessing:
            - type: MULTIPLIER
              parameters: ['0.01']
          tags:
            - {tag: component, value: system}
        - uuid: 0a00010ab2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Node name'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.10.0'
          key: hdc.nodename
          delay: 1h
          value_type: CHAR
          tags:
            - {tag: component, value: system}
        - uuid: 0a00010bb2b2b2b2c3c3c3c3d4d4d4d4
          name: 'MAC address'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.13.0'
          key: hdc.mac
          delay: 1h
          value_type: CHAR
          tags:
            - {tag: component, value: system}
        - uuid: 0a00010cb2b2b2b2c3c3c3c3d4d4d4d4
          name: 'Firmware version'
          type: SNMP_AGENT
          snmp_oid: '1.3.6.1.4.1.55555.1.15.0'
          key: hdc.fwversion
          delay: 1h
          value_type: CHAR
          tags:
            - {tag: component, value: system}
)ZBX";

// Grafana dashboard (import via Dashboards > New > Import). Prompts for a
// Prometheus datasource on import.
static const char GRAFANA_JSON[] PROGMEM = R"GRAF({
  "__inputs": [
    {"name": "DS_PROMETHEUS", "label": "Prometheus", "description": "", "type": "datasource", "pluginId": "prometheus", "pluginName": "Prometheus"}
  ],
  "annotations": {"list": []},
  "editable": true,
  "graphTooltip": 1,
  "tags": ["hdc", "datacenter"],
  "templating": {
    "list": [
      {"name": "instance", "type": "query", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"},
       "query": "label_values(hdc_info, instance)", "refresh": 2, "includeAll": false, "multi": false}
    ]
  },
  "time": {"from": "now-6h", "to": "now"},
  "timepicker": {},
  "schemaVersion": 39,
  "title": "HDC Sense & Control Node",
  "uid": "hdc-node",
  "version": 1,
  "panels": [
    {"id": 1, "title": "Temperature", "type": "timeseries", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"},
     "gridPos": {"h": 8, "w": 12, "x": 0, "y": 0},
     "fieldConfig": {"defaults": {"unit": "celsius"}, "overrides": []},
     "targets": [{"expr": "hdc_temperature_celsius{instance=~\"$instance\"}", "legendFormat": "{{instance}}", "refId": "A", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"}}]},
    {"id": 2, "title": "Humidity", "type": "timeseries", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"},
     "gridPos": {"h": 8, "w": 12, "x": 12, "y": 0},
     "fieldConfig": {"defaults": {"unit": "humidity", "max": 100, "min": 0}, "overrides": []},
     "targets": [{"expr": "hdc_humidity_percent{instance=~\"$instance\"}", "legendFormat": "{{instance}}", "refId": "A", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"}}]},
    {"id": 3, "title": "Airflow (raw ADC)", "type": "timeseries", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"},
     "gridPos": {"h": 8, "w": 12, "x": 0, "y": 8},
     "fieldConfig": {"defaults": {"unit": "none"}, "overrides": []},
     "targets": [{"expr": "hdc_airflow_adc{instance=~\"$instance\"}", "legendFormat": "{{instance}}", "refId": "A", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"}}]},
    {"id": 4, "title": "Relays & Contacts", "type": "state-timeline", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"},
     "gridPos": {"h": 8, "w": 12, "x": 12, "y": 8},
     "fieldConfig": {"defaults": {"max": 1, "min": 0}, "overrides": []},
     "targets": [
       {"expr": "hdc_relay_state{instance=~\"$instance\"}", "legendFormat": "relay {{relay}}", "refId": "A", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"}},
       {"expr": "hdc_contact_state{instance=~\"$instance\"}", "legendFormat": "contact {{contact}}", "refId": "B", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"}}]},
    {"id": 5, "title": "WiFi RSSI", "type": "stat", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"},
     "gridPos": {"h": 6, "w": 8, "x": 0, "y": 16}, "fieldConfig": {"defaults": {"unit": "dBm"}, "overrides": []},
     "targets": [{"expr": "hdc_wifi_rssi_dbm{instance=~\"$instance\"}", "refId": "A", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"}}]},
    {"id": 6, "title": "Uptime", "type": "stat", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"},
     "gridPos": {"h": 6, "w": 8, "x": 8, "y": 16}, "fieldConfig": {"defaults": {"unit": "s"}, "overrides": []},
     "targets": [{"expr": "hdc_uptime_seconds{instance=~\"$instance\"}", "refId": "A", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"}}]},
    {"id": 7, "title": "Free heap", "type": "stat", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"},
     "gridPos": {"h": 6, "w": 8, "x": 16, "y": 16}, "fieldConfig": {"defaults": {"unit": "bytes"}, "overrides": []},
     "targets": [{"expr": "hdc_free_heap_bytes{instance=~\"$instance\"}", "refId": "A", "datasource": {"type": "prometheus", "uid": "${DS_PROMETHEUS}"}}]}
  ]
}
)GRAF";
