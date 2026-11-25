#!/usr/bin/env python3
"""Independent OpenSSL-backed ECDH/HKDF oracle plus production association policy.
No radio or phone access. Clang ASan/UBSan exercise the actual C functions.
"""
from pathlib import Path
import hashlib
import hmac
import os
import random
import subprocess
import sys
import tempfile
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import serialization
root=Path(__file__).resolve().parents[2]
src=root/'kernel/lx/src'
source=(src/'lx_80211.c').read_text()
def function(sig):
    start=source.index(sig);end=source.index('{',start)+1;depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]+'\n'
def array(name,value):
    return 'static const uint8_t '+name+'[]={'+','.join(str(b) for b in value)+'};\n'
rnd=random.Random(int(sys.argv[1]) if len(sys.argv)>1 else 1)
order=0xffffffff00000000ffffffffffffffffbce6faada7179e84f3b9cac2fc632551
fixtures='';entries=[]
for i in range(20):
    # Independently derive public coordinates and the ECDH x via cryptography/OpenSSL.
    a=rnd.randrange(1,order);b=rnd.randrange(1,order)
    own=ec.derive_private_key(a,ec.SECP256R1());peer=ec.derive_private_key(b,ec.SECP256R1())
    c=own.public_key().public_numbers().x.to_bytes(32,'big')
    ap=peer.public_key().public_numbers().x.to_bytes(32,'big')
    shared=own.exchange(ec.ECDH(),peer.public_key())
    prk=hmac.digest(c+ap+b'\x13\0',shared,'sha256')
    pmk=hmac.digest(prk,b'OWE Key Generation\x01','sha256')
    pmkid=hashlib.sha256(c+ap).digest()[:16]
    for name,value in (('secret',a.to_bytes(32,'big')),('pub',c),('peer',ap),('pmk',pmk),('pmkid',pmkid)):
        fixtures+=array(f'f{i}_{name}',value)
    entries.append('{'+','.join(f'f{i}_{name}' for name in ('secret','pub','peer','pmk','pmkid'))+'}')
prefix=r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "lx_owe.h"
#include "lx_wpa.h"
#define EAUTH 80
static void lx_os_lock(void) {}
static void lx_os_unlock(void) {}
struct lx_target { uint8_t elements[1024];unsigned elements_length,capability; };
struct lx_radio {
    struct lx_wpa wpa;struct lx_owe owe;uint8_t rsn[47],sae_pmkid[16];unsigned sae_cached,sae_h2e;
    uint8_t answer_elements[1024];unsigned answer_length;
};
struct fixture { const uint8_t *secret,*pub,*peer,*pmk,*pmkid; };
'''
tests=r'''
static const uint8_t *random_bytes;
static unsigned calls,mode;
static int entropy(void *ctx,uint8_t *p,size_t n) {
    (void)ctx;assert(n==32);calls++;
    if(mode==1) return -1;
    if(mode==2 || (mode==3 && calls==1)) memset(p,0,n);
    else memcpy(p,random_bytes,n);
    return 0;
}
static void setup(struct lx_radio *r,const struct fixture *f) {
    memset(r,0,sizeof(*r));r->wpa.owe=1;r->wpa.group_cipher=4;r->wpa.pmf=1;
    mode=0;calls=0;random_bytes=f->secret;
    assert(!lx_owe_start(&r->owe,entropy,NULL));assert(calls==1);
    assert(!memcmp(r->owe.public_key,f->pub,32));
    personal_elements(r,4);
    assert(r->wpa.rsn_length==28 && r->rsn[19]==18 && r->rsn[20]==128);
    assert(lx_owe_rsn(r->rsn,28));
    memcpy(r->answer_elements,r->rsn,28);
    lx_owe_request(&r->owe,r->answer_elements+28);
    assert(!memcmp(r->answer_elements+28,"\xff\x23\x20\x13\x00",5));
    memcpy(r->answer_elements+33,f->peer,32);r->answer_length=65;
}
int main(void) {
    unsigned checks=0;
    for(unsigned i=0;i<sizeof(fixtures)/sizeof(fixtures[0]);i++) {
        const struct fixture *f=&fixtures[i];struct lx_radio r,initial;setup(&initial,f);
        for(unsigned scenario=0;scenario<14;scenario++) {
            r=initial;
            switch(scenario) {
            case 1:r.answer_length=28;break; /* missing DH */
            case 2:r.answer_elements[31]=20;break; /* unsupported group */
            case 3:memset(r.answer_elements+33,255,32);break; /* x >= p */
            case 4:memset(r.answer_elements+33,0,32);r.answer_elements[64]=1;break; /* non-residue x=1 */
            case 5:r.answer_elements[19]=2;break; /* PSK downgrade */
            case 6:r.answer_elements[20]=0;break; /* PMF removed */
            case 7:memcpy(r.answer_elements+65,r.answer_elements+28,37);r.answer_length+=37;break;
            case 8:r.answer_length--;break; /* truncated DH */
            case 9:r.answer_elements[7]=2;break; /* cipher changed */
            case 10:r.answer_elements[65]=1;r.answer_length=66;break; /* dangling TLV */
            case 11:r.answer_elements[30]=31;break; /* wrong extension */
            case 12:r.answer_length=1025;break; /* bounds */
            case 13:memcpy(r.answer_elements+65,r.answer_elements,28);r.answer_length+=28;break;
            }
            int ret=owe_associate(&r);checks++;
            if(scenario) { assert(ret && !r.wpa.owe_ready);uint8_t zero[32]={0};assert(!memcmp(r.wpa.pmk,zero,32)); }
            else { assert(!ret && r.wpa.owe_ready && !memcmp(r.wpa.pmk,f->pmk,32)); }
            struct lx_owe empty={0};assert(!memcmp(&r.owe,&empty,sizeof(empty)));
        }
        /* Direct PMK/PMKID oracle and one-shot completion. */
        r=initial;uint8_t pmk[32],id[16];
        assert(!lx_owe_finish(&r.owe,r.answer_elements,r.answer_length,pmk,id));
        assert(!memcmp(pmk,f->pmk,32) && !memcmp(id,f->pmkid,16));
        memset(pmk,0xa5,32);assert(lx_owe_finish(&r.owe,r.answer_elements,r.answer_length,pmk,id));assert(pmk[0]==0xa5);
        for(unsigned m=1;m<=3;m++) {
            mode=m;calls=0;int ret=lx_owe_start(&r.owe,entropy,NULL);
            if(m==3) { assert(!ret && calls==2); } else { assert(ret);assert(calls==(m==2?64:1)); }
            lx_owe_clear(&r.owe);checks++;
        }
        /* Every truncation of the RSN fails the predicate. */
        for(unsigned n=0;n<28;n++) assert(!lx_owe_rsn(r.rsn,n));
    }
    printf("OWE: %u association/entropy cases, 20 independent ECDH/PMK/PMKID vectors, malformed TLVs and RSN bounds pass\n",checks);
}
'''
# Raw C string escapes above are ordinary C escapes, not Python byte interpretation.
code=prefix+fixtures+'static const struct fixture fixtures[]={'+','.join(entries)+'};\n'
code+=''.join(function(s) for s in ('static const uint8_t *\nelement(', 'static const char *\nprotection(',
                                   'static void\npersonal_elements(', 'static int\nowe_associate('))+tests
with tempfile.TemporaryDirectory(prefix='hakios-owe-') as d:
    d=Path(d);(d/'test.c').write_text(code)
    subprocess.run([os.environ.get('CC','clang'),'-std=gnu11','-O1','-g','-Wall','-Wextra','-Werror',
                    '-fsanitize=address,undefined','-fno-sanitize-recover=all','-I'+str(src),str(d/'test.c'),
                    *[str(src/s) for s in ('lx_owe.c','lx_sae_crypto.c','lx_sae_math.c')],'-o',str(d/'test')],check=True)
    subprocess.run([str(d/'test')],check=True)
