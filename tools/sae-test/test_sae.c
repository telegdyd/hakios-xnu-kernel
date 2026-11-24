#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lx_sae.c"
#include "ieee.inc"
#include "cases.inc"
static unsigned checks;
#define CHECK(x) do { checks++;if(!(x)){fprintf(stderr,"line %u: %s\n",__LINE__,#x);exit(1);} }while(0)
static int vector_random(void *ctx,uint8_t *out,size_t len)
{ unsigned *i=ctx;if(len!=32||*i>1)return -1;memcpy(out,(*i)++?local_mask:local_rand,32);return 0; }
static int fail_random(void *ctx,uint8_t *out,size_t len) { (void)ctx;(void)out;(void)len;return -1; }
static void unhex(uint8_t *out,const char *hex)
{ for(unsigned i=0;hex[2*i];i++) { unsigned v;CHECK(sscanf(hex+2*i,"%2x",&v)==1);out[i]=(uint8_t)v; } }
struct random_case { const struct sae_case *c;unsigned index; };
static int case_random(void *ctx,uint8_t *out,size_t len)
{ struct random_case *r=ctx;if(len!=32||r->index>1)return -1;memcpy(out,r->index++?r->c->mask:r->c->random,32);return 0; }
int main(void)
{
    struct lx_sae a,b;uint8_t bytes[400],conf[34];unsigned i=0;
    /* RFC 5903 section 8.1, P-256 ECDH common point (not just x). */
    uint8_t private_key[32],peer_point[64],expected_point[64];lx_bn scalar;lx_ec peer_ec,result;
    unhex(private_key,"C88F01F510D9AC3F70A292DAA2316DE544E9AAB8AFE84049C62A9C57862D1433");
    unhex(peer_point,"D12DFB5289C8D4F81208B70270398C342296970A0BCCB74C736FC7554494BF6356FBF3CA366CC23E8157854C13C58D6AAC23F046ADA30F8353E74F33039872AB");
    unhex(expected_point,"D6840F6B42F6EDAFD13116E0E12565202FEF8E9ECE7DCE03812464D04B9442DE522BDE0AF0D8585B8DEF9C183B5AE38F50235206A8674ECB5D98EDB20EB153A2");
    lx_bn_read(&scalar,private_key);CHECK(!lx_ec_read(&peer_ec,peer_point));lx_ec_mul(&result,&peer_ec,&scalar);
    CHECK(!lx_ec_write(bytes,&result));CHECK(!memcmp(bytes,expected_point,64));
    scalar=(lx_bn){{0}};lx_ec_mul(&result,&peer_ec,&scalar);CHECK(lx_ec_write(bytes,&result)==-1);
    lx_ec_mul(&result,&peer_ec,&lx_p256_n);CHECK(lx_ec_write(bytes,&result)==-1);
    CHECK(!lx_sae_start(&a,addr1,addr2,(const uint8_t *)"byteme",6,(const uint8_t *)"mekmitasdigoat",14,0,vector_random,&i));
    CHECK(lx_sae_write_commit(&a,bytes)==98);CHECK(!memcmp(bytes,local_commit,98));
    CHECK(lx_sae_peer_commit(&a,bytes,98)==-1); /* reflection */
    for(unsigned n=0;n<98;n++) CHECK(lx_sae_peer_commit(&a,peer_commit,n)==-1);
    CHECK(!lx_sae_peer_commit(&a,peer_commit,98));CHECK(!memcmp(a.kck,kck,32));CHECK(!memcmp(a.pmk,pmk,32));CHECK(!memcmp(a.pmkid,pmkid,16));
    CHECK(!lx_sae_write_confirm(&a,conf));
    b=a;memcpy(b.commit,a.peer,98);memcpy(b.peer,a.commit,98);b.send_confirm=0;
    CHECK(!lx_sae_write_confirm(&b,conf));conf[3]^=1;CHECK(lx_sae_check_confirm(&a,conf,34)==-1);CHECK(a.state==2);conf[3]^=1;
    CHECK(!lx_sae_check_confirm(&a,conf,34));CHECK(a.state==3);CHECK(lx_sae_check_confirm(&a,conf,34)==-1);
    CHECK(!lx_sae_peer_commit(&a,peer_commit,98));bytes[2]^=1;CHECK(lx_sae_peer_commit(&a,bytes,98)==-1);
    lx_ec p={0};uint8_t addrs[12];addresses(addrs,addr1b,addr2b);
    CHECK(!hash_to_element(&p,addrs,(const uint8_t *)"byteme",6,(const uint8_t *)"mekmitasdigoatpsk4internet",26));
    CHECK(!lx_ec_write(bytes,&p));CHECK(!memcmp(bytes,pwe_19_x,32));CHECK(!memcmp(bytes+32,pwe_19_y,32));
    struct lx_sae_cache cache={0};
    lx_sae_cache_store(&cache,&a,addr1,addr2,(const uint8_t *)"byteme",6,(const uint8_t *)"pw",2,10);
    CHECK(lx_sae_cache_get(&cache,addr1,addr2,(const uint8_t *)"byteme",6,(const uint8_t *)"pw",2,0,11,bytes,bytes+32)==1);
    CHECK(lx_sae_cache_get(&cache,addr1,addr2,(const uint8_t *)"byteme",6,(const uint8_t *)"px",2,0,11,bytes,bytes+32)==0);
    CHECK(lx_sae_cache_get(&cache,addr1,addr2,(const uint8_t *)"byteme",6,(const uint8_t *)"pw",2,1,11,bytes,bytes+32)==0);
    CHECK(lx_sae_cache_get(&cache,addr1,addr2,(const uint8_t *)"byteme",6,(const uint8_t *)"pw",2,0,cache.expires,bytes,bytes+32)==0);CHECK(!cache.valid);
    CHECK(lx_sae_start(&a,addr1,addr2,(const uint8_t *)"byteme",6,(const uint8_t *)"pw",2,1,fail_random,NULL)==-1);
    CHECK(!a.state);lx_sae_clear(&a);memset(&b,0,sizeof(b));CHECK(!memcmp(&a,&b,sizeof(a)));
    for(unsigned c=0;c<sizeof(math_cases)/sizeof(math_cases[0]);c++) {
        const struct math_case *m=&math_cases[c];lx_bn x,y,z;const lx_bn *mod=m->mod?&lx_p256_n:&lx_p256_p;
        lx_bn_read(&x,m->a);lx_bn_read(&y,m->b);
        lx_bn_add(&z,&x,&y,mod);lx_bn_write(bytes,&z);CHECK(!memcmp(bytes,m->sum,32));
        lx_bn_sub(&z,&x,&y,mod);lx_bn_write(bytes,&z);CHECK(!memcmp(bytes,m->diff,32));
        lx_bn_mul(&z,&x,&y,mod);lx_bn_write(bytes,&z);CHECK(!memcmp(bytes,m->product,32));
    }
    for(unsigned c=0;c<sizeof(cases)/sizeof(cases[0]);c++) {
        const struct sae_case *v=&cases[c];struct random_case rc={v,0};
        CHECK(!lx_sae_start(&a,v->own,v->peer,(const uint8_t *)"testssid",8,v->password,16,v->h2e,case_random,&rc));
        CHECK(!lx_ec_write(bytes,&a.pwe));CHECK(!memcmp(bytes,v->pwe,64));
        CHECK(lx_sae_write_commit(&a,bytes)==98);CHECK(!memcmp(bytes,v->commit,98));
        uint8_t token[]={19,0,255,5,93,1,2,3,4};
        CHECK(!lx_sae_token(&a,token,v->h2e?sizeof(token):6));
        unsigned wirelen=lx_sae_write_commit(&a,bytes);
        CHECK(wirelen==(v->h2e?105u:102u));
        CHECK(!memcmp(bytes+(v->h2e?2:6),v->commit+2,96));
        CHECK(lx_sae_token(&a,token,2)==-1);CHECK(lx_sae_token(&a,token,400)==-1);
        memcpy(bytes,v->peer_commit,98);memset(bytes+34,0,64);CHECK(lx_sae_peer_commit(&a,bytes,98)==-1);
        memcpy(bytes,v->peer_commit,98);memset(bytes+2,0,32);CHECK(lx_sae_peer_commit(&a,bytes,98)==-1);
        bytes[33]=1;CHECK(lx_sae_peer_commit(&a,bytes,98)==-1);
        lx_bn_write(bytes+2,&lx_p256_n);CHECK(lx_sae_peer_commit(&a,bytes,98)==-1);
        CHECK(!lx_sae_peer_commit(&a,v->peer_commit,98));CHECK(!memcmp(a.kck,v->kck,32));CHECK(!memcmp(a.pmk,v->pmk,32));CHECK(!memcmp(a.pmkid,v->pmkid,16));
        CHECK(!lx_sae_check_confirm(&a,v->confirm,34));CHECK(lx_sae_check_confirm(&a,v->confirm,34)==-1);
        for(unsigned n=0;n<34;n++) CHECK(lx_sae_check_confirm(&a,v->confirm,n)==-1);
        lx_sae_clear(&a);
    }
    printf("sae: %u checks, 0 failed\n",checks);return 0;
}
