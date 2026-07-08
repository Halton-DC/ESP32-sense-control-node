/**
 * certs.h - Self-signed TLS certificate management.
 *
 * On first boot (or after factory reset) a fresh EC P-256 key and a
 * self-signed certificate are generated on-device and stored in NVS. No
 * private key is ever embedded in the firmware image, so every unit has a
 * unique certificate. Subsequent boots reuse the stored pair.
 */
#pragma once
#include <Arduino.h>
#include <stdint.h>

// Ensure a cert/key pair exists (generating + persisting if needed).
// `cn` is used as the certificate Common Name (e.g. the node name).
// Returns true if a valid pair is available.
bool certsBegin(const char *cn);

// Accessors for the PEM material. Lengths INCLUDE the trailing NUL byte,
// as required by esp_https_server.
const uint8_t *certsServerCert(size_t *len);
const uint8_t *certsPrivateKey(size_t *len);
