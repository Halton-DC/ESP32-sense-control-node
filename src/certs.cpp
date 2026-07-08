#include "certs.h"
#include <Preferences.h>
#include <string.h>
#include "mbedtls/pk.h"
#include "mbedtls/ecp.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"

static const char *NVS_NS   = "hdc";
static const char *NVS_CERT = "tlscert";
static const char *NVS_PKEY = "tlskey";

// Held for the lifetime of the process so the HTTPS server can reference them.
static uint8_t s_cert[1400];
static size_t  s_certLen = 0;
static uint8_t s_key[600];
static size_t  s_keyLen = 0;

const uint8_t *certsServerCert(size_t *len) { *len = s_certLen; return s_cert; }
const uint8_t *certsPrivateKey(size_t *len) { *len = s_keyLen; return s_key; }

// Generate a fresh EC key + self-signed cert into the static buffers.
static bool generate(const char *cn) {
  int ret = 0;
  mbedtls_pk_context key;
  mbedtls_x509write_cert crt;
  mbedtls_entropy_context entropy;
  mbedtls_ctr_drbg_context ctr_drbg;
  mbedtls_pk_init(&key);
  mbedtls_x509write_crt_init(&crt);
  mbedtls_entropy_init(&entropy);
  mbedtls_ctr_drbg_init(&ctr_drbg);

  const char *pers = "hdc_tls_gen";
  char subject[96];
  snprintf(subject, sizeof(subject), "CN=%s,O=Halton Datacenter", cn);

  bool ok = false;
  do {
    if (mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                              (const unsigned char *)pers, strlen(pers)) != 0) break;

    if (mbedtls_pk_setup(&key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY)) != 0) break;
    if (mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(key),
                            mbedtls_ctr_drbg_random, &ctr_drbg) != 0) break;

    // Private key -> PEM
    unsigned char keyPem[600];
    if (mbedtls_pk_write_key_pem(&key, keyPem, sizeof(keyPem)) != 0) break;
    s_keyLen = strlen((char *)keyPem) + 1;
    if (s_keyLen > sizeof(s_key)) break;
    memcpy(s_key, keyPem, s_keyLen);

    // Self-signed certificate
    mbedtls_x509write_crt_set_subject_key(&crt, &key);
    mbedtls_x509write_crt_set_issuer_key(&crt, &key);
    if (mbedtls_x509write_crt_set_subject_name(&crt, subject) != 0) break;
    if (mbedtls_x509write_crt_set_issuer_name(&crt, subject) != 0) break;
    mbedtls_x509write_crt_set_version(&crt, MBEDTLS_X509_CRT_VERSION_3);
    mbedtls_x509write_crt_set_md_alg(&crt, MBEDTLS_MD_SHA256);

    unsigned char serial[16];
    mbedtls_ctr_drbg_random(&ctr_drbg, serial, sizeof(serial));
    serial[0] &= 0x7F; // keep it positive
    if (mbedtls_x509write_crt_set_serial_raw(&crt, serial, sizeof(serial)) != 0) break;

    // ~20 year validity (device has no RTC at gen time; self-signed anyway).
    if (mbedtls_x509write_crt_set_validity(&crt, "20240101000000", "20440101000000") != 0) break;
    mbedtls_x509write_crt_set_basic_constraints(&crt, 0, -1); // not a CA

    unsigned char certPem[1400];
    ret = mbedtls_x509write_crt_pem(&crt, certPem, sizeof(certPem),
                                    mbedtls_ctr_drbg_random, &ctr_drbg);
    if (ret != 0) break;
    s_certLen = strlen((char *)certPem) + 1;
    if (s_certLen > sizeof(s_cert)) break;
    memcpy(s_cert, certPem, s_certLen);

    ok = true;
  } while (0);

  mbedtls_x509write_crt_free(&crt);
  mbedtls_pk_free(&key);
  mbedtls_ctr_drbg_free(&ctr_drbg);
  mbedtls_entropy_free(&entropy);
  return ok;
}

bool certsBegin(const char *cn) {
  Preferences prefs;
  prefs.begin(NVS_NS, false);

  size_t cl = prefs.getBytesLength(NVS_CERT);
  size_t kl = prefs.getBytesLength(NVS_PKEY);
  if (cl > 0 && cl <= sizeof(s_cert) && kl > 0 && kl <= sizeof(s_key)) {
    s_certLen = prefs.getBytes(NVS_CERT, s_cert, sizeof(s_cert));
    s_keyLen  = prefs.getBytes(NVS_PKEY, s_key, sizeof(s_key));
    prefs.end();
    Serial.println("[TLS] Loaded certificate from NVS.");
    return s_certLen > 0 && s_keyLen > 0;
  }

  Serial.println("[TLS] Generating self-signed certificate (first boot)...");
  uint32_t t0 = millis();
  if (!generate(cn)) {
    Serial.println("[TLS] Certificate generation FAILED.");
    prefs.end();
    return false;
  }
  prefs.putBytes(NVS_CERT, s_cert, s_certLen);
  prefs.putBytes(NVS_PKEY, s_key, s_keyLen);
  prefs.end();
  Serial.printf("[TLS] Certificate generated and stored (%lu ms).\n",
                (unsigned long)(millis() - t0));
  return true;
}
