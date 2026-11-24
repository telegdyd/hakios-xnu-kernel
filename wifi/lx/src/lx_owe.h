/* RFC 8110, group 19. No PMK caching; a new exchange on every join. */
#ifndef LX_OWE_H
#define LX_OWE_H
#include "lx_sae_crypto.h"
struct lx_owe { lx_bn secret; uint8_t public_key[32]; unsigned active; };
int lx_owe_start(struct lx_owe *, int (*entropy)(void *, uint8_t *, size_t), void *);
void lx_owe_clear(struct lx_owe *);
void lx_owe_request(const struct lx_owe *, uint8_t out[37]);
int lx_owe_finish(struct lx_owe *, const uint8_t *elements, size_t length, uint8_t pmk[32], uint8_t pmkid[16]);
/* True only for a bounded RSN with exactly the OWE AKM. Full cipher/PMF policy is the MLME's. */
int lx_owe_rsn(const uint8_t *rsn, size_t length);
#endif
