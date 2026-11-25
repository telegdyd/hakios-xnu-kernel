/* SAE station, group 19. No OS calls: caller supplies entropy and serializes.
 * Authentication bodies exclude algorithm/transaction/status (the MLME owns those).
 * Public lengths are bounded; no password, scalar or PMK is logged.
 */
#ifndef LX_SAE_H
#define LX_SAE_H
#include "lx_sae_crypto.h"
#define LX_SAE_COMMIT 98
#define LX_SAE_CONFIRM 34
#define LX_SAE_TOKEN_MAX 256
struct lx_sae {
    lx_bn random;
    lx_ec pwe;
    uint8_t commit[LX_SAE_COMMIT], peer[LX_SAE_COMMIT];
    uint8_t kck[32], pmk[32], pmkid[16];
    uint8_t token[LX_SAE_TOKEN_MAX];
    unsigned token_length, h2e, state;
    uint16_t send_confirm, receive_confirm;
};
typedef int (*lx_sae_random)(void *, uint8_t *, size_t);
int lx_sae_start(struct lx_sae *,const uint8_t own[6],const uint8_t peer[6],
    const uint8_t *ssid,size_t ssid_length,const uint8_t *password,size_t password_length,
    unsigned h2e,lx_sae_random,void *);
/* Returns 0 after validated peer commit and key derivation, -1 on rejection.
 * Repeating an identical commit keeps the keys; reflection is always rejected. */
int lx_sae_peer_commit(struct lx_sae *,const uint8_t *,size_t);
/* Token retry preserves the original commit/random; H2E uses token-container IE. */
int lx_sae_token(struct lx_sae *,const uint8_t *,size_t);
unsigned lx_sae_write_commit(const struct lx_sae *,uint8_t *);
int lx_sae_write_confirm(struct lx_sae *,uint8_t [LX_SAE_CONFIRM]);
int lx_sae_check_confirm(struct lx_sae *,const uint8_t *,size_t);
void lx_sae_clear(struct lx_sae *);

/* One bounded station cache. Credential tag prevents reuse after a password
 * change; monotonic time only. Cache entries must also bind the selected AKM. */
struct lx_sae_cache {
    uint8_t own[6],peer[6],ssid[32],tag[32],pmk[32],pmkid[16];
    unsigned ssid_length,h2e,valid;
    uint64_t expires;
};
void lx_sae_cache_store(struct lx_sae_cache *,const struct lx_sae *,const uint8_t [6],const uint8_t [6],
    const uint8_t *,size_t,const uint8_t *,size_t,uint64_t);
int lx_sae_cache_get(struct lx_sae_cache *,const uint8_t [6],const uint8_t [6],const uint8_t *,size_t,
    const uint8_t *,size_t,unsigned,uint64_t,uint8_t [32],uint8_t [16]);
#endif
