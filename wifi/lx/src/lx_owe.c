/* OWE group-19 ECDH and HKDF, RFC 8110 sections 4.3-4.4.
 * Reuses the fixed-size P-256 backend; no borrowed hostap implementation. */
#include "lx_owe.h"
#ifdef _KERNEL
#include <sys/param.h>
#include <sys/systm.h>
#else
#include <string.h>
#endif
void lx_owe_clear(struct lx_owe *s) { lx_crypto_wipe(s, sizeof(*s)); }
int lx_owe_start(struct lx_owe *s, int (*entropy)(void *, uint8_t *, size_t), void *ctx)
{
    /* SEC 2 P-256 generator x; either y sign gives the same compact ECDH. */
    static const uint8_t gx[32]={0x6b,0x17,0xd1,0xf2,0xe1,0x2c,0x42,0x47,0xf8,0xbc,0xe6,0xe5,0x63,0xa4,0x40,0xf2,
        0x77,0x03,0x7d,0x81,0x2d,0xeb,0x33,0xa0,0xf4,0xa1,0x39,0x45,0xd8,0x98,0xc2,0x96};
    struct { lx_bn x; lx_ec g,p; uint8_t bytes[64]; } w={0};
    int ret=-1;
    lx_owe_clear(s);
    for (unsigned tries=0; tries<64; tries++) {
        if (entropy(ctx,w.bytes,32)) goto out;
        lx_bn_read(&s->secret,w.bytes);
        if (lx_bn_zero(&s->secret) || !lx_bn_less(&s->secret,&lx_p256_n)) continue;
        lx_bn_read(&w.x,gx);
        if (!lx_ec_from_x(&w.g,&w.x,0)) goto out;
        lx_ec_mul(&w.p,&w.g,&s->secret);
        if (lx_ec_write(w.bytes,&w.p)) goto out;
        memcpy(s->public_key,w.bytes,32);s->active=1;ret=0;break;
    }
out:
    lx_crypto_wipe(&w,sizeof(w));if(ret) lx_owe_clear(s);return ret;
}
void lx_owe_request(const struct lx_owe *s, uint8_t out[37])
{ out[0]=255;out[1]=35;out[2]=32;out[3]=19;out[4]=0;memcpy(out+5,s->public_key,32); }
int lx_owe_rsn(const uint8_t *r,size_t n)
{
    size_t at=8,count;
    if(!r || n<10 || r[0]!=48 || (size_t)r[1]+2!=n || r[2]!=1 || r[3]) return 0;
    count=r[at]|r[at+1]<<8;at+=2;
    if(!count || count>(n-at)/4) return 0;
    at+=4*count;if(n-at<2) return 0;
    count=r[at]|r[at+1]<<8;at+=2;
    return count==1 && n-at>=4 && !memcmp(r+at,"\x00\x0f\xac\x12",4);
}
int lx_owe_finish(struct lx_owe *s,const uint8_t *ies,size_t n,uint8_t pmk[32],uint8_t pmkid[16])
{
    struct { lx_bn x;lx_ec peer,shared;uint8_t point[64],salt[66],prk[32],hash[32],pmk[32]; } w={0};
    const uint8_t *dh=NULL;unsigned rsn_count=0;int ret=-1;
    if(!s->active) goto out;
    while(n) {
        if(n<2 || (size_t)ies[1]+2>n) goto out;
        size_t len=(size_t)ies[1]+2;
        if(ies[0]==48 && ++rsn_count>1) goto out;
        if(ies[0]==255 && ies[1] && ies[2]==32) {
            if(dh || len!=37 || ies[3]!=19 || ies[4]) goto out;
            dh=ies+5;
        }
        ies+=len;n-=len;
    }
    if(!dh) goto out;
    lx_bn_read(&w.x,dh);
    if(!lx_ec_from_x(&w.peer,&w.x,0)) goto out;
    lx_ec_mul(&w.shared,&w.peer,&s->secret);
    if(lx_ec_write(w.point,&w.shared)) goto out;
    memcpy(w.salt,s->public_key,32);memcpy(w.salt+32,dh,32);w.salt[64]=19;w.salt[65]=0;
    const uint8_t *v[]={w.point};size_t z[]={32};
    lx_hmac256(w.salt,66,1,v,z,w.prk);
    lx_hkdf256(w.prk,"OWE Key Generation",w.pmk,32);
    v[0]=w.salt;z[0]=64;lx_sha256_vector(1,v,z,w.hash);
    memcpy(pmk,w.pmk,32);memcpy(pmkid,w.hash,16);ret=0;
out:
    lx_crypto_wipe(&w,sizeof(w));lx_owe_clear(s);return ret;
}
