#!/usr/bin/env python3
"""Compile the production RSN parser and key lifecycle against a recording driver.

Only OS allocation/locking and the driver are replaced. Run with Python 3 and
Clang; ASan/UBSan check malformed RSN input and key replacement/cleanup.
"""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'wifi/lx/src/lx_80211.c').read_text()


def function(signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define GFP_KERNEL 0
#define WLAN_CIPHER_SUITE_TKIP 0x000fac02
#define WLAN_CIPHER_SUITE_CCMP 0x000fac04
#define IEEE80211_KEY_FLAG_PAIRWISE 1
#define SET_KEY 1
#define DISABLE_KEY 2
#define __containerof(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define mutex_lock(p) ((void)(p))
#define mutex_unlock(p) ((void)(p))
#define lx_tell(...) ((void)0)
struct ieee80211_key_conf {
    unsigned cipher, keyidx, keylen, iv_len, icv_len, flags;
    int link_id;
    uint8_t key[];
};
struct work_struct { int dummy; };
struct driver { int (*set_key)(void *, int, void *, void *, struct ieee80211_key_conf *); };
struct lx_radio {
    struct { unsigned group_cipher, group_index; uint8_t group[32]; } wpa;
    struct { int mtx; } *wiphy;
    struct driver *ops;
    int hw;
    void *vif, *sta;
    struct ieee80211_key_conf *group_key;
    struct work_struct group_work;
    unsigned rekeys;
};
struct lx_target { uint8_t *elements; unsigned elements_length, capability; };
static int fail_alloc, driver_error, sets, disables, freed;
static void *last_sta;
static struct { unsigned cipher, len, iv, icv, flags, index; uint8_t key[32]; } installed;
static void *kzalloc(size_t n, int flags) {
    (void)flags;
    return fail_alloc ? NULL : calloc(1,n);
}
static void kfree(void *p) {
    struct ieee80211_key_conf *key = p;
    for (unsigned i=0; i<key->keylen; i++) assert(key->key[i] == 0);
    freed++;
    free(p);
}
static int set_key(void *hw, int cmd, void *vif, void *sta, struct ieee80211_key_conf *key) {
    (void)hw; (void)vif;
    last_sta = sta;
    if (cmd == DISABLE_KEY) { disables++; return 0; }
    sets++;
    installed.cipher=key->cipher; installed.len=key->keylen;
    installed.iv=key->iv_len; installed.icv=key->icv_len;
    installed.flags=key->flags; installed.index=key->keyidx;
    memcpy(installed.key,key->key,key->keylen);
    return driver_error;
}
'''

tests = r'''
int main(void) {
    uint8_t rsn[] = {48,20,1,0,0,15,172,2,1,0,0,15,172,4,1,0,0,15,172,2,0,0};
    struct lx_target target = {rsn,sizeof(rsn),0x10};
    unsigned cipher=0;
    assert(protection(&target,&cipher)==NULL && cipher==2);
    rsn[7]=4;
    assert(protection(&target,&cipher)==NULL && cipher==4);
    rsn[7]=1; assert(protection(&target,&cipher)!=NULL); rsn[7]=2;
    rsn[2]=2; assert(protection(&target,&cipher)!=NULL); rsn[2]=1;
    rsn[8]=0; assert(protection(&target,&cipher)!=NULL); rsn[8]=1;
    rsn[8]=255; assert(protection(&target,&cipher)!=NULL); rsn[8]=1;
    rsn[14]=0; assert(protection(&target,&cipher)!=NULL); rsn[14]=1;
    rsn[14]=255; assert(protection(&target,&cipher)!=NULL); rsn[14]=1;
    rsn[20]=0x40; assert(protection(&target,&cipher)!=NULL); rsn[20]=0;
    for (unsigned n=0; n<sizeof(rsn); n++) {
        target.elements_length=n;
        assert(protection(&target,&cipher)!=NULL);
    }
    target.elements_length=sizeof(rsn);
    /* Try every possible declared length, including truncated cipher/AKM lists. */
    for (unsigned n=0; n<256; n++) { rsn[1]=n; (void)protection(&target,&cipher); }
    struct driver ops={set_key};
    struct lx_radio radio={0};
    radio.ops=&ops;
    radio.sta=&radio;
    radio.wiphy=calloc(1,sizeof(*radio.wiphy));
    for (unsigned c=2; c<=4; c+=2) {
        radio.wpa.group_cipher=c; radio.wpa.group_index=1;
        for (unsigned n=0; n<32; n++) radio.wpa.group[n]=n+1;
        int error=0;
        struct ieee80211_key_conf *pair=key_set(&radio,radio.wpa.group,0,true,&error);
        assert(error==0 && installed.cipher==WLAN_CIPHER_SUITE_CCMP && installed.len==16);
        assert(installed.flags==IEEE80211_KEY_FLAG_PAIRWISE && last_sta==radio.sta);
        key_drop(&radio,&pair,true); assert(pair==NULL);
        group_rekey(&radio.group_work);
        assert(radio.group_key && installed.len==(c==2 ? 32u:16u));
        assert(installed.cipher==(c==2 ? WLAN_CIPHER_SUITE_TKIP:WLAN_CIPHER_SUITE_CCMP));
        assert(installed.iv==8 && installed.icv==(c==2 ? 4u:8u));
        assert(installed.flags==0 && last_sta==NULL);
        assert(memcmp(installed.key,radio.wpa.group,installed.len)==0);
        int before=disables;
        group_rekey(&radio.group_work); assert(disables==before); /* same-index replacement */
        radio.wpa.group_index=2;
        group_rekey(&radio.group_work); assert(disables==before+1);
        struct ieee80211_key_conf *old=radio.group_key;
        driver_error=-EIO;
        group_rekey(&radio.group_work); assert(radio.group_key==old && disables==before+1);
        driver_error=0; fail_alloc=1;
        group_rekey(&radio.group_work); assert(radio.group_key==old && disables==before+1);
        fail_alloc=0;
        key_drop(&radio,&radio.group_key,false); assert(radio.group_key==NULL);
    }
    free(radio.wiphy);
    printf("station: RSN bounds, CCMP/TKIP driver keys, same/different-index rekey, failures and %d wiped frees pass (%d SET, %d DISABLE)\n",freed,sets,disables);
}
'''

with tempfile.TemporaryDirectory(prefix='hakios-station-') as tmp:
    tmp = Path(tmp)
    code = prefix + ''.join(function(s) for s in (
        'static const uint8_t *\nelement(', 'static const char *\nprotection(',
        'static struct ieee80211_key_conf *\nkey_set(', 'static void\nkey_drop(',
        'static void\ngroup_rekey(')) + tests
    (tmp / 'test.c').write_text(code)
    subprocess.run([os.environ.get('CC', 'clang'), '-std=gnu11', '-O1', '-g',
                    '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                    '-fno-sanitize-recover=all', str(tmp / 'test.c'), '-o', str(tmp / 'test')], check=True)
    subprocess.run([str(tmp / 'test')], check=True)
