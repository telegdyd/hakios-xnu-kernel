#!/usr/bin/env python3
"""Production management RX/TX against a recording driver; ASan/UBSan, no phone."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'wifi/lx/src/lx_80211.c').read_text()

def function(signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'

prefix = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "lx_wpa.c"
#define ETH_ALEN 6
#define ELEMENTS_MOST 256
#define IEEE80211_STA_ASSOC 3
#define HAKIOS_STA_LOSS_DEAUTH 3
#define HAKIOS_STA_LOSS_DISASSOC 4
#define RX_FLAG_DECRYPTED 1
#define RX_FLAG_IV_STRIPPED 2
#define RX_FLAG_MIC_STRIPPED 4
#define RX_FLAG_PN_VALIDATED 8
#define NL80211_BAND_2GHZ 0
#define IEEE80211_TX_CTL_HW_80211_ENCAP 1
#define IEEE80211_SKB_CB(s) (&(s)->info)
#define lx_tell(...) ((void)0)
#define bzero(p,n) memset(p,0,n)
#define ECONNRESET 54
struct ieee80211_hw { int dummy; };
struct ieee80211_tx_info { struct { void *vif, *hw_key; } control; int band; unsigned flags; };
struct sk_buff { unsigned len; uint8_t *data; uint8_t bytes[512]; struct ieee80211_tx_info info; };
struct ieee80211_tx_control { void *sta; };
struct ieee80211_rx_status { unsigned flag; };
struct ops { void (*tx)(struct ieee80211_hw *, struct ieee80211_tx_control *, struct sk_buff *); };
struct work_struct { int dummy; };
struct lx_radio {
    struct lx_wpa wpa;
    bool sae_listening;
    unsigned sae_rx_length;
    uint8_t sae_rx[400];
    uint64_t management_tx_pn;
    struct ieee80211_hw hw;
    struct ops *ops;
    void *sta, *pairwise_key, *work;
    struct { uint8_t addr[6]; } *vif;
    struct { struct { struct { int band; } *chan; } def; } *channel;
    struct { uint8_t address[6]; } target;
    int reached, joined, answer, last_error;
    unsigned loss, loss_reason;
    unsigned aid, answer_length, answer_status;
    uint8_t answer_elements[ELEMENTS_MOST];
    struct { int start, amsdu, active; unsigned size, token, timeout, ssn; } ba[8];
    unsigned ba_asked;
    struct work_struct ba_work;
};
enum { ANSWER_NONE, ANSWER_AUTHENTICATED, ANSWER_ASSOCIATED, ANSWER_REFUSED, ANSWER_GONE };
static unsigned sent, downs, lost;
static uint8_t transmitted[512];
static unsigned transmitted_length;
static void *transmitted_key;
static int lx_os_option(const char *s) { (void)s; return 0; }
static void lx_os_lock(void) {}
static void lx_os_unlock(void) {}
static void lx_os_wakeup(void *p) { (void)p; }
static void lx_os_net_link(int up) { if (!up) downs++; }
static void lx_os_station_lost(int e) { (void)e; lost++; }
static void queue_work(void *q, struct work_struct *w) { (void)q; (void)w; }
static struct sk_buff *dev_alloc_skb(unsigned n) { assert(n<=512); return calloc(1,sizeof(struct sk_buff)); }
static void skb_reserve(struct sk_buff *s,unsigned n) { s->data=s->bytes+n; }
static void *skb_put(struct sk_buff *s,unsigned n) { void *p=s->data+s->len; s->len+=n; assert(s->data+s->len<=s->bytes+512); return p; }
static void dev_kfree_skb_any(struct sk_buff *s) { free(s); }
static void tx(struct ieee80211_hw *hw,struct ieee80211_tx_control *c,struct sk_buff *s) {
    (void)hw; (void)c; sent++;
    memcpy(transmitted,s->data,s->len); transmitted_length=s->len;
    transmitted_key=s->info.control.hw_key; free(s);
}
"""

tests = r"""
static void request(uint8_t *f,unsigned pn) {
    memset(f,0,44); f[0]=0xd0; f[1]=0x40;
    memset(f+4,2,6); memset(f+10,4,12);
    f[24]=pn; f[27]=0x20;
    f[32]=8; f[33]=0; f[34]=0x12; f[35]=0x34;
}
int main(void) {
    struct lx_radio r={0}; struct ops ops={tx};
    struct ieee80211_rx_status rx={RX_FLAG_DECRYPTED};
    r.ops=&ops; r.sta=&r; r.pairwise_key=&ops;
    r.vif=calloc(1,sizeof(*r.vif)); memset(r.vif->addr,2,6); memset(r.target.address,4,6);
    r.wpa.pmf=1; r.wpa.replay_known=1; r.reached=IEEE80211_STA_ASSOC; r.joined=1;
    uint8_t f[64]; request(f,1);
    management_received(&r,f,44,&rx);
    assert(sent==1 && transmitted_length==36 && transmitted[0]==0xd0 && transmitted[1]==0x40);
    assert(transmitted[24]==1 && transmitted[27]==0x20 && transmitted[32]==8 && transmitted[33]==1);
    assert(transmitted[34]==0x12 && transmitted[35]==0x34 && transmitted_key==r.pairwise_key);
    request(f,1); management_received(&r,f,44,&rx); assert(sent==1); /* replay */
    request(f,2); f[10]=6; management_received(&r,f,44,&rx); assert(sent==1); /* wrong sender */
    request(f,2); f[16]=6; management_received(&r,f,44,&rx); assert(sent==1); /* wrong BSSID */
    request(f,2); f[4]=6; management_received(&r,f,44,&rx); assert(sent==1); /* wrong destination */
    request(f,2); f[0]=0xc0; f[1]=0; f[24]=3;
    management_received(&r,f,26,&rx); assert(r.joined && !downs); /* forged deauth */
    request(f,2); f[0]=0xa0; f[1]=0; f[24]=3;
    management_received(&r,f,26,&rx); assert(r.joined && !downs); /* forged disassoc */
    request(f,2); f[0]=0xc0; f[32]=3;
    management_received(&r,f,42,&rx);
    assert(!r.joined && downs==1 && lost==1 && r.answer==ANSWER_GONE && r.answer_status==3);
    /* A leave's deauth uses the same protected TX path and a fresh PN. */
    uint8_t *body; struct sk_buff *s=frame_new(&r,26,&body);
    body=management(&r,body,0xc0); body[0]=3; body[1]=0; frame_send(&r,s);
    assert(sent==2 && transmitted_length==34 && transmitted[24]==2 && transmitted[32]==3);
    /* Public action frames remain unprotected. */
    s=frame_new(&r,26,&body); body=management(&r,body,0xd0); body[0]=4; body[1]=0; frame_send(&r,s);
    assert(sent==3 && transmitted_length==26 && !(transmitted[1]&0x40) && !transmitted_key);
    /* An Ethernet destination beginning with a management subtype must never enter PMF TX. */
    s=frame_new(&r,26,&body); memset(body,0,26); body[0]=0xc0;
    s->info.flags=IEEE80211_TX_CTL_HW_80211_ENCAP; frame_send(&r,s);
    assert(sent==4 && transmitted_length==26 && !transmitted_key);
    r.management_tx_pn=(1ull<<48)-1;
    s=frame_new(&r,26,&body); body=management(&r,body,0xc0); body[0]=3; body[1]=0; frame_send(&r,s);
    assert(sent==4); /* exhausted PN cannot wrap */
    /* A broadcast deauth needs a valid MMIE and must reach the same link-down path. */
    r.joined=1; memset(f,0,44); f[0]=0xc0; memset(f+4,0xff,6); memset(f+10,4,12); f[24]=3;
    f[26]=76; f[27]=16; f[28]=4; f[30]=1; r.wpa.igtk[0].valid=1;
    uint8_t aad[20],tag[16]; aad[0]=0xc0; aad[1]=0; memcpy(aad+2,f+4,18);
    const uint8_t *parts[]={aad,f+24}; const unsigned lengths[]={20,20};
    cmac(r.wpa.igtk[0].key,parts,lengths,2,tag); memcpy(f+36,tag,8);
    f[36]^=1; management_received(&r,f,44,&rx); assert(r.joined && downs==1);
    f[36]^=1; management_received(&r,f,44,&rx); assert(!r.joined && downs==2 && lost==2);
    /* Legacy WPA2 still accepts an unprotected deauth. */
    r.wpa.pmf=0; r.joined=1; request(f,3); f[0]=0xc0; f[1]=0; f[24]=3; f[25]=0;
    management_received(&r,f,26,&rx); assert(!r.joined && downs==3);
    free(r.vif);
    puts("PMF station: protected SA Query TX, spoof/replay drops, authenticated deauth, legacy deauth and PN exhaustion pass");
}
"""

with tempfile.TemporaryDirectory(prefix='hakios-pmf-station-') as tmp:
    tmp = Path(tmp)
    code = prefix + ''.join(function(s) for s in (
        'static struct sk_buff *\nframe_new(', 'static void\nframe_send(',
        'static uint8_t *\nmanagement(', 'static void\nmanagement_received(')) + tests
    (tmp / 'test.c').write_text(code)
    subprocess.run([os.environ.get('CC', 'clang'), '-std=gnu11', '-O1', '-g',
                    '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                    '-fno-sanitize-recover=all', '-I'+str(root/'wifi/lx/src'), str(tmp/'test.c'),
                    str(root/'wifi/lx/src/lx_sae_crypto.c'), '-o', str(tmp/'test')], check=True)
    subprocess.run([str(tmp/'test')], check=True)
