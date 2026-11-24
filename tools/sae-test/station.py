#!/usr/bin/env python3
"""Run production SAE MLME and management RX with a recording/simulated AP.
The crypto fixtures come from the independent affine Python oracle. Reuse the
PMF harness's OS/driver declarations so the two management tests stay aligned.
"""
from pathlib import Path
import ast
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root/'wifi/lx/src/lx_80211.c').read_text()
def function(signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth, end = 1, brace+1
    while depth:
        depth += (source[end] == '{')-(source[end] == '}'); end += 1
    return source[start:end]+'\n'

tree = ast.parse((root/'tools/wpa-test/pmf_station.py').read_text())
prefix = next(ast.literal_eval(node.value) for node in tree.body if isinstance(node,ast.Assign)
              and any(isinstance(t,ast.Name) and t.id=='prefix' for t in node.targets))
prefix = prefix.replace('#include "lx_wpa.c"', '#include "lx_wpa.c"\n#include "lx_owe.h"\n#include "lx_sae.h"\n#include <errno.h>\n#define EAUTH 80')
prefix = prefix.replace('#define ECONNRESET 54', '')
prefix = prefix.replace('    bool sae_listening;', '    struct lx_owe owe; struct lx_sae sae; unsigned sae_h2e,sae_cached; uint8_t rsn[47],sae_pmkid[16];\n    bool sae_listening;')
prefix = prefix.replace('static void tx(', 'static void simulate(const uint8_t *,unsigned);\nstatic void tx(')
prefix = prefix.replace('    transmitted_key=s->info.control.hw_key; free(s);',
                        '    transmitted_key=s->info.control.hw_key; simulate(s->data,s->len); free(s);')
prefix += r'''
struct hakios_sta_join { uint8_t ssid[32];unsigned ssid_length;uint8_t passphrase[64];unsigned passphrase_length; };
static uint64_t now;
static uint64_t lx_os_nanoseconds(void) { return now; }
static void lx_os_sleep(void *event,uint64_t duration) { (void)event;now+=duration; }
#include "cases.inc"
static const struct sae_case *chosen;
static unsigned entropy_index;
static void get_random_bytes(uint8_t *p,size_t n) { assert(n==32 && entropy_index<2);memcpy(p,entropy_index++?chosen->mask:chosen->random,32); }
'''
tests = r'''
static struct lx_radio *active;
static unsigned scenario, commits, confirms;
static void feed(unsigned seq,unsigned status,const uint8_t *body,unsigned len) {
    uint8_t f[440]={0};struct ieee80211_rx_status rx={0};f[0]=0xb0;
    memcpy(f+4,chosen->own,6);memcpy(f+10,chosen->peer,6);memcpy(f+16,chosen->peer,6);
    f[24]=3;f[26]=seq;f[28]=status;memcpy(f+30,body,len);
    /* Neither another algorithm nor a multicast auth response can enter the mailbox. */
    f[24]=0;management_received(active,f,30+len,&rx);assert(!active->sae_rx_length);f[24]=3;
    f[4]|=1;management_received(active,f,30+len,&rx);assert(!active->sae_rx_length);memcpy(f+4,chosen->own,6);
    management_received(active,f,30+len,&rx);
}
static void simulate(const uint8_t *f,unsigned n) {
    assert(n>=30 && f[0]==0xb0 && f[24]==3 && f[25]==0);
    assert(!memcmp(f+4,chosen->peer,6));
    if(f[26]==1) {
        commits++;assert(f[28]==(chosen->h2e?126:0));
        unsigned offset=(scenario==1 && commits>1 && !chosen->h2e)?4:0;
        assert(!memcmp(f+32+offset,chosen->commit+2,96));
        if(scenario==1 && commits==1) {
            uint8_t token[]={19,0,255,5,93,1,2,3,4};
            feed(1,76,token,chosen->h2e?sizeof(token):6);return;
        }
        if(scenario==2) { feed(1,77,chosen->peer_commit,2);return; }
        if(scenario==4) return; /* lost commits */
        feed(1,chosen->h2e?126:0,chosen->peer_commit,98);
    } else {
        assert(f[26]==2 && f[28]==0 && n==64);confirms++;
        if(scenario==5 && confirms==1) { feed(1,chosen->h2e?126:0,chosen->peer_commit,98);return; }
        uint8_t conf[34];memcpy(conf,chosen->confirm,34);if(scenario==3) conf[15]^=1;
        feed(2,0,conf,34);
    }
}
int main(void) {
    for(unsigned c=0;c<sizeof(cases)/sizeof(cases[0]);c++) {
        /* Management addresses are unicast on the wire. Fixtures used for the
         * arithmetic tests can be arbitrary; skip multicast addresses here. */
        chosen=&cases[c];if(chosen->own[0]&1) continue;
        for(scenario=0;scenario<6;scenario++) {
            struct lx_radio r={0};struct ops ops={tx};struct hakios_sta_join what={0};
            r.vif=calloc(1,sizeof(*r.vif));memcpy(r.vif->addr,chosen->own,6);memcpy(r.target.address,chosen->peer,6);
            r.sta=&r;r.ops=&ops;r.sae_h2e=chosen->h2e;r.answer=ANSWER_GONE;active=&r;commits=confirms=entropy_index=0;now=0;
            memcpy(what.ssid,"testssid",8);what.ssid_length=8;memcpy(what.passphrase,chosen->password,16);what.passphrase_length=16;
            int ret=sae_authenticate(&r,&what);
            if(scenario==2 || scenario==3 || scenario==4) { assert(ret!=0 && !r.sae.state);uint8_t z[32]={0};assert(!memcmp(r.wpa.pmk,z,32)); }
            else { assert(ret==0 && r.sae.state==3);assert(!memcmp(r.wpa.pmk,chosen->pmk,32)); }
            r.wpa.sae=1;r.wpa.pmf=1;personal_elements(&r,4);
            assert(r.rsn[19]==8 && r.rsn[20]==0xc0 && r.rsn[1]==26);
            assert(r.wpa.rsn_length==(chosen->h2e?31u:28u));
            if(chosen->h2e) assert(!memcmp(r.rsn+28,"\xf4\x01\x20",3));
            r.sae_cached=1;memcpy(r.sae_pmkid,chosen->pmkid,16);personal_elements(&r,4);
            assert(r.rsn[1]==42 && r.rsn[22]==1 && !memcmp(r.rsn+24,chosen->pmkid,16));
            assert(r.wpa.rsn_length==(chosen->h2e?47u:44u));
            assert(!r.sae_listening && !r.sae_rx_length && entropy_index==2);
            assert(commits+confirms<=8);if(scenario==1) assert(commits==2);
            lx_sae_clear(&r.sae);free(r.vif);
        }
    }
    puts("SAE station: production RX/MLME, token retries, duplicate commits, initial-zero confirms, refusal, timeout and forged-confirm rejection pass");
}
'''
with tempfile.TemporaryDirectory(prefix='hakios-sae-station-') as tmp:
    tmp = Path(tmp)
    with (tmp/'cases.inc').open('w') as out:
        subprocess.run(['python3',str(root/'tools/sae-test/reference.py'),'1'],stdout=out,check=True)
    code = prefix+''.join(function(s) for s in (
        'static struct sk_buff *\nframe_new(', 'static void\nframe_send(',
        'static uint8_t *\nmanagement(', 'static void\nmanagement_received(',
        'static void\npersonal_elements(', 'static int\nsae_entropy(', 'static int\nsae_authenticate('))+tests
    (tmp/'test.c').write_text(code)
    subprocess.run([os.environ.get('CC','clang'),'-std=gnu11','-O1','-g','-Wall','-Wextra','-Werror',
                    '-fsanitize=address,undefined','-fno-sanitize-recover=all',
                    '-I'+str(root/'wifi/lx/src'),'-I'+str(tmp),str(tmp/'test.c'),
                    *[str(root/'wifi/lx/src'/s) for s in ('lx_owe.c','lx_sae.c','lx_sae_math.c','lx_sae_crypto.c')],
                    '-o',str(tmp/'test')],check=True)
    subprocess.run([str(tmp/'test')],check=True)
