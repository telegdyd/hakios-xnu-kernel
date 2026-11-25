/* Group 19 only, no allocation. Field/scalar arithmetic has fixed iteration
 * counts and masked selection, including reduction, inverse and square root.
 * Jacobian formulas: EFD dbl-2001-b / add-1998-cmo, a=-3. Exceptional cases
 * are selected after computing both results, never branched on secret data.
 * SAE SSWU follows hostap 2.11 sae.c (BSD; sae-crypto/HOSTAP-LICENSE).
 * Copyright (c) 2012-2024 Jouni Malinen and contributors (SSWU adaptation).
 */
#include "lx_sae_crypto.h"
#define BN(a,b,c,d,e,f,g,h) {{a,b,c,d,e,f,g,h}}
const lx_bn lx_p256_p = BN(0xffffffff,0xffffffff,0xffffffff,0,0,0,1,0xffffffff);
const lx_bn lx_p256_n = BN(0xfc632551,0xf3b9cac2,0xa7179e84,0xbce6faad,0xffffffff,0xffffffff,0,0xffffffff);
static const lx_bn zero = {{0}}, one = {{1}};
static const lx_bn curve_b = BN(0x27d2604b,0x3bce3c3e,0xcc53b0f6,0x651d06b0,0x769886bc,0xb3ebbd55,0xaa3a93e7,0x5ac635d8);
static const lx_bn sqrt_exp = BN(0,0,0x40000000,0,0,0x40000000,0xc0000000,0x3fffffff);
static const lx_bn inv_exp = BN(0xfffffffd,0xffffffff,0xffffffff,0,0,0,1,0xffffffff);

static uint32_t sub_raw(lx_bn *r, const lx_bn *a, const lx_bn *b)
{
    uint64_t borrow = 0;
    for (unsigned i=0;i<8;i++) { uint64_t t=(uint64_t)a->v[i]-b->v[i]-borrow; r->v[i]=(uint32_t)t; borrow=t>>63; }
    return (uint32_t)borrow;
}
int lx_bn_zero(const lx_bn *a) { uint32_t v=0; for(unsigned i=0;i<8;i++) v|=a->v[i]; return (int)(((v | (0u-v))>>31)^1); }
int lx_bn_less(const lx_bn *a, const lx_bn *b) { lx_bn t; return (int)sub_raw(&t,a,b); }
void lx_bn_select(lx_bn *r,const lx_bn *a,const lx_bn *b,unsigned choose_a)
{ uint32_t mask=0u-choose_a; for(unsigned i=0;i<8;i++) r->v[i]=(a->v[i]&mask)|(b->v[i]&~mask); }
void lx_bn_read(lx_bn *r,const uint8_t b[32])
{ for(unsigned i=0;i<8;i++) { unsigned j=28-4*i; r->v[i]=(uint32_t)b[j]<<24|(uint32_t)b[j+1]<<16|(uint32_t)b[j+2]<<8|b[j+3]; } }
void lx_bn_write(uint8_t b[32],const lx_bn *a)
{ for(unsigned i=0;i<8;i++) { unsigned j=28-4*i; uint32_t v=a->v[i]; b[j]=v>>24;b[j+1]=v>>16;b[j+2]=v>>8;b[j+3]=v; } }
void lx_bn_reduce(lx_bn *r,const uint8_t *bytes,size_t len,const lx_bn *m)
{
    lx_bn a=zero,t;
    for(size_t i=0;i<len;i++) for(int bit=7;bit>=0;bit--) {
        uint32_t carry=(bytes[i]>>bit)&1;
        for(unsigned j=0;j<8;j++) { uint32_t next=a.v[j]>>31; a.v[j]=(a.v[j]<<1)|carry;carry=next; }
        uint32_t borrow=sub_raw(&t,&a,m);
        lx_bn_select(&a,&t,&a,carry|(borrow^1));
    }
    *r=a;lx_crypto_wipe(&a,sizeof(a));lx_crypto_wipe(&t,sizeof(t));
}
void lx_bn_add(lx_bn *r,const lx_bn *a,const lx_bn *b,const lx_bn *m)
{
    lx_bn s,t; uint64_t carry=0;
    for(unsigned i=0;i<8;i++) { uint64_t v=(uint64_t)a->v[i]+b->v[i]+carry;s.v[i]=(uint32_t)v;carry=v>>32; }
    uint32_t borrow=sub_raw(&t,&s,m);
    lx_bn_select(r,&t,&s,(unsigned)carry|(borrow^1));
    lx_crypto_wipe(&s,sizeof(s));lx_crypto_wipe(&t,sizeof(t));
}
void lx_bn_sub(lx_bn *r,const lx_bn *a,const lx_bn *b,const lx_bn *m)
{
    lx_bn t; uint32_t mask=0u-sub_raw(&t,a,b);uint64_t carry=0;
    for(unsigned i=0;i<8;i++) { uint64_t v=(uint64_t)t.v[i]+(m->v[i]&mask)+carry;r->v[i]=(uint32_t)v;carry=v>>32; }
    lx_crypto_wipe(&t,sizeof(t));
}
void lx_bn_mul(lx_bn *r,const lx_bn *a,const lx_bn *b,const lx_bn *m)
{
    uint32_t product[16]={0};uint8_t bytes[64];
    for(unsigned i=0;i<8;i++) { uint64_t carry=0;
        for(unsigned j=0;j<8;j++) { uint64_t v=(uint64_t)a->v[i]*b->v[j]+product[i+j]+carry;product[i+j]=(uint32_t)v;carry=v>>32; }
        product[i+8]=(uint32_t)carry;
    }
    for(unsigned i=0;i<16;i++) { unsigned j=60-4*i;uint32_t v=product[i];bytes[j]=v>>24;bytes[j+1]=v>>16;bytes[j+2]=v>>8;bytes[j+3]=v; }
    lx_bn_reduce(r,bytes,64,m);lx_crypto_wipe(product,sizeof(product));lx_crypto_wipe(bytes,sizeof(bytes));
}
void lx_bn_pow(lx_bn *r,const lx_bn *a,const lx_bn *e,const lx_bn *m)
{
    lx_bn s=one,t;
    for(int i=255;i>=0;i--) { lx_bn_mul(&s,&s,&s,m);lx_bn_mul(&t,&s,a,m);lx_bn_select(&s,&t,&s,(e->v[i/32]>>(i%32))&1); }
    *r=s;lx_crypto_wipe(&t,sizeof(t));lx_crypto_wipe(&s,sizeof(s));
}
#define add(r,a,b) lx_bn_add(r,a,b,&lx_p256_p)
#define sub(r,a,b) lx_bn_sub(r,a,b,&lx_p256_p)
#define mul(r,a,b) lx_bn_mul(r,a,b,&lx_p256_p)
static void inv(lx_bn *r,const lx_bn *a) { lx_bn_pow(r,a,&inv_exp,&lx_p256_p); }
static void ec_select(lx_ec *r,const lx_ec *a,const lx_ec *b,unsigned yes)
{ lx_bn_select(&r->x,&a->x,&b->x,yes);lx_bn_select(&r->y,&a->y,&b->y,yes);lx_bn_select(&r->z,&a->z,&b->z,yes); }
static void ec_double(lx_ec *r,const lx_ec *p)
{
    lx_bn delta,gamma,beta,alpha,t,u;lx_ec q;
    mul(&delta,&p->z,&p->z);mul(&gamma,&p->y,&p->y);mul(&beta,&p->x,&gamma);
    sub(&t,&p->x,&delta);add(&u,&p->x,&delta);mul(&alpha,&t,&u);add(&t,&alpha,&alpha);add(&alpha,&alpha,&t);
    add(&t,&beta,&beta);add(&t,&t,&t);add(&u,&t,&t);mul(&q.x,&alpha,&alpha);sub(&q.x,&q.x,&u);
    add(&u,&p->y,&p->z);mul(&q.z,&u,&u);sub(&q.z,&q.z,&gamma);sub(&q.z,&q.z,&delta);
    sub(&t,&t,&q.x);mul(&q.y,&alpha,&t);mul(&u,&gamma,&gamma);add(&u,&u,&u);add(&u,&u,&u);add(&u,&u,&u);sub(&q.y,&q.y,&u);
    *r=q;lx_crypto_wipe(&q,sizeof(q));lx_crypto_wipe(&delta,sizeof(delta));lx_crypto_wipe(&gamma,sizeof(gamma));lx_crypto_wipe(&beta,sizeof(beta));lx_crypto_wipe(&alpha,sizeof(alpha));lx_crypto_wipe(&t,sizeof(t));lx_crypto_wipe(&u,sizeof(u));
}
void lx_ec_add(lx_ec *r,const lx_ec *p,const lx_ec *q)
{
    struct { lx_bn z1,z2,u1,u2,s1,s2,h,d,h2,h3,t; lx_ec out,doubled,infinity; } w={0};
    mul(&w.z1,&p->z,&p->z);mul(&w.z2,&q->z,&q->z);
    mul(&w.u1,&p->x,&w.z2);mul(&w.u2,&q->x,&w.z1);
    mul(&w.t,&q->z,&w.z2);mul(&w.s1,&p->y,&w.t);mul(&w.t,&p->z,&w.z1);mul(&w.s2,&q->y,&w.t);
    sub(&w.h,&w.u2,&w.u1);sub(&w.d,&w.s2,&w.s1);mul(&w.h2,&w.h,&w.h);mul(&w.h3,&w.h2,&w.h);
    mul(&w.t,&w.u1,&w.h2);mul(&w.out.x,&w.d,&w.d);sub(&w.out.x,&w.out.x,&w.h3);sub(&w.out.x,&w.out.x,&w.t);sub(&w.out.x,&w.out.x,&w.t);
    sub(&w.t,&w.t,&w.out.x);mul(&w.out.y,&w.d,&w.t);mul(&w.t,&w.s1,&w.h3);sub(&w.out.y,&w.out.y,&w.t);
    mul(&w.t,&p->z,&q->z);mul(&w.out.z,&w.t,&w.h);
    ec_double(&w.doubled,p);
    ec_select(&w.out,&w.infinity,&w.out,lx_bn_zero(&w.h));
    ec_select(&w.out,&w.doubled,&w.out,lx_bn_zero(&w.h)&lx_bn_zero(&w.d));
    ec_select(&w.out,q,&w.out,lx_bn_zero(&p->z));ec_select(&w.out,p,&w.out,lx_bn_zero(&q->z));
    *r=w.out;lx_crypto_wipe(&w,sizeof(w));
}
void lx_ec_mul(lx_ec *r,const lx_ec *p,const lx_bn *k)
{
    lx_ec a={0},b;
    for(int i=255;i>=0;i--) { ec_double(&a,&a);lx_ec_add(&b,&a,p);ec_select(&a,&b,&a,(k->v[i/32]>>(i%32))&1); }
    *r=a;lx_crypto_wipe(&a,sizeof(a));lx_crypto_wipe(&b,sizeof(b));
}
static void y_squared(lx_bn *r,const lx_bn *x)
{ lx_bn t,u;mul(&t,x,x);mul(&t,&t,x);add(&u,x,x);add(&u,&u,x);sub(&t,&t,&u);add(r,&t,&curve_b);lx_crypto_wipe(&t,sizeof(t));lx_crypto_wipe(&u,sizeof(u)); }
int lx_ec_from_x(lx_ec *p,const lx_bn *x,unsigned parity)
{
    lx_bn g,y,check,neg; y_squared(&g,x);lx_bn_pow(&y,&g,&sqrt_exp,&lx_p256_p);
    mul(&check,&y,&y);sub(&check,&check,&g);sub(&neg,&zero,&y);
    p->x=*x;lx_bn_select(&p->y,&neg,&y,(y.v[0]^parity)&1);p->z=one;
    int ok=lx_bn_zero(&check)&lx_bn_less(x,&lx_p256_p);
    lx_crypto_wipe(&g,sizeof(g));lx_crypto_wipe(&y,sizeof(y));lx_crypto_wipe(&check,sizeof(check));lx_crypto_wipe(&neg,sizeof(neg));return ok;
}
int lx_ec_read(lx_ec *r,const uint8_t bytes[64])
{
    lx_bn x,y,g,t;lx_bn_read(&x,bytes);lx_bn_read(&y,bytes+32);y_squared(&g,&x);mul(&t,&y,&y);sub(&t,&t,&g);
    if(!lx_bn_less(&x,&lx_p256_p)||!lx_bn_less(&y,&lx_p256_p)||!lx_bn_zero(&t)) return -1;
    r->x=x;r->y=y;r->z=one;return 0;
}
int lx_ec_write(uint8_t bytes[64],const lx_ec *p)
{
    lx_bn z,z2,x,y;inv(&z,&p->z);mul(&z2,&z,&z);mul(&x,&p->x,&z2);mul(&z2,&z2,&z);mul(&y,&p->y,&z2);
    lx_bn_write(bytes,&x);lx_bn_write(bytes+32,&y);
    lx_crypto_wipe(&z,sizeof(z));lx_crypto_wipe(&z2,sizeof(z2));lx_crypto_wipe(&x,sizeof(x));lx_crypto_wipe(&y,sizeof(y));
    return lx_bn_zero(&p->z)?-1:0;
}
void lx_ec_negate(lx_ec *p) { sub(&p->y,&zero,&p->y); }
void lx_ec_sswu(lx_ec *p,const lx_bn *u)
{
    /* Simplified SWU, Z=-10, A=-3. RFC 9380 section 6.6.2; hostap sae.c. */
    struct { lx_bn z,a,u2,t,m,x1,x2,g1,g2,y1,y2,b_over_za,minus_b_over_a,neg;lx_ec p1,p2; } w={0};
    w.z.v[0]=10;sub(&w.z,&zero,&w.z);w.a.v[0]=3;sub(&w.a,&zero,&w.a);
    mul(&w.t,&w.z,&w.a);inv(&w.t,&w.t);mul(&w.b_over_za,&curve_b,&w.t);
    inv(&w.t,&w.a);mul(&w.minus_b_over_a,&curve_b,&w.t);sub(&w.minus_b_over_a,&zero,&w.minus_b_over_a);
    mul(&w.u2,u,u);mul(&w.t,&w.z,&w.u2);mul(&w.m,&w.t,&w.t);add(&w.m,&w.m,&w.t);
    inv(&w.t,&w.m);add(&w.t,&one,&w.t);mul(&w.x1,&w.minus_b_over_a,&w.t);
    lx_bn_select(&w.x1,&w.b_over_za,&w.x1,lx_bn_zero(&w.m));
    mul(&w.t,&w.z,&w.u2);mul(&w.x2,&w.t,&w.x1);
    unsigned qr=(unsigned)lx_ec_from_x(&w.p1,&w.x1,u->v[0]&1);
    (void)lx_ec_from_x(&w.p2,&w.x2,u->v[0]&1);ec_select(p,&w.p1,&w.p2,qr);
    lx_crypto_wipe(&w,sizeof(w));
}
