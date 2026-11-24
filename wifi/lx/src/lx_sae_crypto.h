/* Fixed-size group 19 backend. Integers and points on the wire are big endian. */
#ifndef LX_SAE_CRYPTO_H
#define LX_SAE_CRYPTO_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t v[8]; } lx_bn;
typedef struct { lx_bn x, y, z; } lx_ec;
extern const lx_bn lx_p256_p, lx_p256_n;
void lx_crypto_wipe(void *, size_t);
int lx_crypto_equal(const void *, const void *, size_t);
void lx_sha256_vector(unsigned, const uint8_t *const *, const size_t *, uint8_t [32]);
void lx_hmac256(const uint8_t *, size_t, unsigned, const uint8_t *const *, const size_t *, uint8_t [32]);
void lx_kdf256(const uint8_t [32], const char *, const uint8_t *, size_t, uint8_t *, size_t);
void lx_hkdf256(const uint8_t [32], const char *, uint8_t *, size_t);
void lx_bn_read(lx_bn *, const uint8_t [32]);
void lx_bn_write(uint8_t [32], const lx_bn *);
void lx_bn_reduce(lx_bn *, const uint8_t *, size_t, const lx_bn *);
int lx_bn_zero(const lx_bn *);
int lx_bn_less(const lx_bn *, const lx_bn *);
void lx_bn_select(lx_bn *, const lx_bn *, const lx_bn *, unsigned);
void lx_bn_add(lx_bn *, const lx_bn *, const lx_bn *, const lx_bn *);
void lx_bn_sub(lx_bn *, const lx_bn *, const lx_bn *, const lx_bn *);
void lx_bn_mul(lx_bn *, const lx_bn *, const lx_bn *, const lx_bn *);
void lx_bn_pow(lx_bn *, const lx_bn *, const lx_bn *, const lx_bn *);
void lx_ec_add(lx_ec *, const lx_ec *, const lx_ec *);
void lx_ec_mul(lx_ec *, const lx_ec *, const lx_bn *);
int lx_ec_read(lx_ec *, const uint8_t [64]);
int lx_ec_write(uint8_t [64], const lx_ec *);
void lx_ec_negate(lx_ec *);
/* Point from x, requested y parity; returns 1 only for an on-curve point. */
int lx_ec_from_x(lx_ec *, const lx_bn *, unsigned);
void lx_ec_sswu(lx_ec *, const lx_bn *);
#endif
