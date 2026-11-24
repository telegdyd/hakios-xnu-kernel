/* SAE group 19 station. Adapted from hostap 2.11 src/common/sae.c:
 * Copyright (c) 2012-2024 Jouni Malinen <j@w1.fi> and contributors.
 * BSD-3-Clause, full notice in sae-crypto/HOSTAP-LICENSE.
 * Only P-256/SHA256, no FFC, password identifier, SAE-PK or extended-key AKM.
 */
#ifdef _KERNEL
#include <sys/param.h>
#include <sys/systm.h>
#else
#include <string.h>
#endif
#include "lx_sae.h"
static const lx_bn one={{1}};
void lx_sae_clear(struct lx_sae *s) { lx_crypto_wipe(s,sizeof(*s)); }
static void addresses(uint8_t out[12],const uint8_t a[6],const uint8_t b[6])
{ int first=memcmp(a,b,6)>0;memcpy(out,first?a:b,6);memcpy(out+6,first?b:a,6); }
static int random_scalar(lx_bn *v,lx_sae_random rng,void *ctx)
{
    uint8_t bytes[32];int ok=-1;
    for(unsigned tries=0;tries<64;tries++) {
        if(rng(ctx,bytes,32)!=0) break;
        lx_bn_read(v,bytes);
        if(!lx_bn_zero(v)&&lx_bn_less(v,&lx_p256_n)) { ok=0;break; }
    }
    lx_crypto_wipe(bytes,sizeof(bytes));return ok;
}
static int hunting(lx_ec *p,const uint8_t addrs[12],const uint8_t *pw,size_t pwlen)
{
    struct { uint8_t prime[32],seed[32],value[32];lx_bn x;lx_ec candidate; } w={0};
    unsigned found=0;lx_bn_write(w.prime,&lx_p256_p);
    /* Always 40 complete field/sqrt tests and masked selection of the first
     * valid point. Bound failure probability <2^-40; fail closed if exhausted. */
    for(uint8_t counter=1;counter<=40;counter++) {
        const uint8_t *v[]={pw,&counter};size_t n[]={pwlen,1};
        lx_hmac256(addrs,12,2,v,n,w.seed);lx_kdf256(w.seed,"SAE Hunting and Pecking",w.prime,32,w.value,32);
        lx_bn_read(&w.x,w.value);unsigned in_range=lx_bn_less(&w.x,&lx_p256_p);
        /* The arithmetic accepts only reduced operands, even on a failed candidate. */
        lx_bn_reduce(&w.x,w.value,32,&lx_p256_p);
        unsigned valid=(unsigned)lx_ec_from_x(&w.candidate,&w.x,w.seed[31]&1)&in_range;
        unsigned take=valid&(found^1);
        lx_bn_select(&p->x,&w.candidate.x,&p->x,take);lx_bn_select(&p->y,&w.candidate.y,&p->y,take);lx_bn_select(&p->z,&w.candidate.z,&p->z,take);
        found|=valid;
    }
    lx_crypto_wipe(&w,sizeof(w));return found?0:-1;
}
static int hash_to_element(lx_ec *p,const uint8_t addrs[12],const uint8_t *ssid,size_t ssidlen,const uint8_t *pw,size_t pwlen)
{
    struct { uint8_t seed[32],value[48],hash[32],salt[32];lx_bn u,nminus1,val;lx_ec p1,p2,pt; } w={0};
    const uint8_t *v[]={pw};size_t n[]={pwlen};lx_hmac256(ssid,ssidlen,1,v,n,w.seed);
    lx_hkdf256(w.seed,"SAE Hash to Element u1 P1",w.value,48);lx_bn_reduce(&w.u,w.value,48,&lx_p256_p);lx_ec_sswu(&w.p1,&w.u);
    lx_hkdf256(w.seed,"SAE Hash to Element u2 P2",w.value,48);lx_bn_reduce(&w.u,w.value,48,&lx_p256_p);lx_ec_sswu(&w.p2,&w.u);
    lx_ec_add(&w.pt,&w.p1,&w.p2);
    v[0]=addrs;n[0]=12;lx_hmac256(w.salt,32,1,v,n,w.hash);
    lx_bn_sub(&w.nminus1,&lx_p256_n,&one,&lx_p256_n);lx_bn_reduce(&w.val,w.hash,32,&w.nminus1);
    lx_bn_add(&w.val,&w.val,&one,&lx_p256_n);lx_ec_mul(p,&w.pt,&w.val);
    int ok=lx_bn_zero(&p->z)?-1:0;lx_crypto_wipe(&w,sizeof(w));return ok;
}
int lx_sae_start(struct lx_sae *s,const uint8_t own[6],const uint8_t peer[6],const uint8_t *ssid,size_t ssidlen,
    const uint8_t *pw,size_t pwlen,unsigned h2e,lx_sae_random rng,void *ctx)
{
    struct { uint8_t addrs[12];lx_bn mask,scalar;lx_ec element; } w={0};int ret=-1;
    lx_sae_clear(s);
    if(!ssidlen||ssidlen>32||!pwlen||pwlen>63||!rng||h2e>1||!memcmp(own,peer,6)) return -1;
    addresses(w.addrs,own,peer);s->h2e=h2e;
    if((h2e?hash_to_element(&s->pwe,w.addrs,ssid,ssidlen,pw,pwlen):hunting(&s->pwe,w.addrs,pw,pwlen))!=0) goto out;
    for(unsigned tries=0;tries<64;tries++) {
        if(random_scalar(&s->random,rng,ctx)||random_scalar(&w.mask,rng,ctx)) goto out;
        lx_bn_add(&w.scalar,&s->random,&w.mask,&lx_p256_n);
        if(lx_bn_less(&one,&w.scalar)) { ret=0;break; }
    }
    if(ret) goto out;
    lx_ec_mul(&w.element,&s->pwe,&w.mask);lx_ec_negate(&w.element);
    s->commit[0]=19;s->commit[1]=0;lx_bn_write(s->commit+2,&w.scalar);
    if(lx_ec_write(s->commit+34,&w.element)) { ret=-1;goto out; }
    s->state=1;
out:
    lx_crypto_wipe(&w,sizeof(w));if(ret) lx_sae_clear(s);return ret;
}
int lx_sae_peer_commit(struct lx_sae *s,const uint8_t *body,size_t len)
{
    struct { lx_bn scalar,own,sum;lx_ec element,t,k;uint8_t point[64],seed[32],context[32],keys[64],salt[32]; } w={0};int ret=-1;
    if(!s->state||len!=98||body[0]!=19||body[1]!=0||lx_crypto_equal(body,s->commit,98)) return -1;
    /* A changed commit cannot replace keys after the first accepted commit. */
    if(s->state>=2) return lx_crypto_equal(body,s->peer,98)?0:-1;
    lx_bn_read(&w.scalar,body+2);
    if(!lx_bn_less(&one,&w.scalar)||!lx_bn_less(&w.scalar,&lx_p256_n)||lx_ec_read(&w.element,body+34)) goto out;
    lx_ec_mul(&w.t,&s->pwe,&w.scalar);lx_ec_add(&w.t,&w.t,&w.element);lx_ec_mul(&w.k,&w.t,&s->random);
    if(lx_ec_write(w.point,&w.k)) goto out;
    const uint8_t *v[]={w.point};size_t n[]={32};lx_hmac256(w.salt,32,1,v,n,w.seed);
    lx_bn_read(&w.own,s->commit+2);lx_bn_add(&w.sum,&w.own,&w.scalar,&lx_p256_n);lx_bn_write(w.context,&w.sum);
    lx_kdf256(w.seed,"SAE KCK and PMK",w.context,32,w.keys,64);
    memcpy(s->kck,w.keys,32);memcpy(s->pmk,w.keys+32,32);memcpy(s->pmkid,w.context,16);memcpy(s->peer,body,98);
    s->state=2;ret=0;
    lx_crypto_wipe(&s->random,sizeof(s->random));lx_crypto_wipe(&s->pwe,sizeof(s->pwe));
out:
    lx_crypto_wipe(&w,sizeof(w));return ret;
}
int lx_sae_token(struct lx_sae *s,const uint8_t *body,size_t len)
{
    if(s->state!=1||len<3||body[0]!=19||body[1]!=0) return -1;
    body+=2;len-=2;
    /* H2E token-container extension element 93; extension payload max 254. */
    if(s->h2e) { if(len<4||body[0]!=255||body[1]+2u!=len||body[2]!=93) return -1;len-=3;body+=3; }
    if(!len||len>LX_SAE_TOKEN_MAX) return -1;
    memcpy(s->token,body,len);s->token_length=(unsigned)len;return 0;
}
unsigned lx_sae_write_commit(const struct lx_sae *s,uint8_t *out)
{
    unsigned n=0;if(s->state!=1&&s->state!=2) return 0;
    memcpy(out,s->commit,2);n=2;
    if(!s->h2e&&s->token_length) { memcpy(out+n,s->token,s->token_length);n+=s->token_length; }
    memcpy(out+n,s->commit+2,96);n+=96;
    if(s->h2e&&s->token_length) { out[n++]=255;out[n++]=(uint8_t)(s->token_length+1);out[n++]=93;memcpy(out+n,s->token,s->token_length);n+=s->token_length; }
    return n;
}
static void confirm(const struct lx_sae *s,const uint8_t counter[2],const uint8_t *first,const uint8_t *second,uint8_t out[32])
{ const uint8_t *v[]={counter,first+2,first+34,second+2,second+34};size_t n[]={2,32,64,32,64};lx_hmac256(s->kck,32,5,v,n,out); }
int lx_sae_write_confirm(struct lx_sae *s,uint8_t out[34])
{
    if(s->state<2||s->send_confirm==65535) return -1;
    s->send_confirm++;out[0]=(uint8_t)s->send_confirm;out[1]=(uint8_t)(s->send_confirm>>8);confirm(s,out,s->commit,s->peer,out+2);return 0;
}
int lx_sae_check_confirm(struct lx_sae *s,const uint8_t *body,size_t len)
{
    uint8_t expected[32];if(s->state<2||len!=34) return -1;
    uint16_t counter=(uint16_t)(body[0]|body[1]<<8);
    if(s->state==3 && counter<=s->receive_confirm) return -1;
    confirm(s,body,s->peer,s->commit,expected);int ok=lx_crypto_equal(expected,body+2,32);lx_crypto_wipe(expected,sizeof(expected));
    if(!ok) return -1;
    s->receive_confirm=counter;s->state=3;return 0;
}
static void credential_tag(uint8_t tag[32],const uint8_t *pw,size_t len)
{ const uint8_t *v[]={pw};size_t n[]={len};lx_sha256_vector(1,v,n,tag); }
void lx_sae_cache_store(struct lx_sae_cache *c,const struct lx_sae *s,const uint8_t own[6],const uint8_t peer[6],
    const uint8_t *ssid,size_t len,const uint8_t *pw,size_t pwlen,uint64_t now)
{
    lx_crypto_wipe(c,sizeof(*c));if(s->state!=3||!len||len>32) return;
    memcpy(c->own,own,6);memcpy(c->peer,peer,6);memcpy(c->ssid,ssid,len);c->ssid_length=(unsigned)len;
    credential_tag(c->tag,pw,pwlen);memcpy(c->pmk,s->pmk,32);memcpy(c->pmkid,s->pmkid,16);c->h2e=s->h2e;
    c->expires=now+3600000000000ull;c->valid=1;
}
int lx_sae_cache_get(struct lx_sae_cache *c,const uint8_t own[6],const uint8_t peer[6],const uint8_t *ssid,size_t len,
    const uint8_t *pw,size_t pwlen,unsigned h2e,uint64_t now,uint8_t pmk[32],uint8_t pmkid[16])
{
    uint8_t tag[32];if(!c->valid) return 0;
    if(now>=c->expires) { lx_crypto_wipe(c,sizeof(*c));return 0; }
    credential_tag(tag,pw,pwlen);
    int ok=len==c->ssid_length&&h2e==c->h2e&&lx_crypto_equal(own,c->own,6)&&lx_crypto_equal(peer,c->peer,6)&&lx_crypto_equal(ssid,c->ssid,len)&&lx_crypto_equal(tag,c->tag,32);
    lx_crypto_wipe(tag,sizeof(tag));if(!ok) return 0;
    memcpy(pmk,c->pmk,32);memcpy(pmkid,c->pmkid,16);return 1;
}
