"""Exercise the production link-PM register ordering, restoration and failures."""
from pathlib import Path
import subprocess,tempfile
r=Path(__file__).resolve().parents[1]
prefix=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define CHECK(x) do {int e=(x);if(e)return e;}while(0)
static struct pm_port {unsigned bdf,pc,ss,lc,ctl,sta,dc2,dctl2,sc,s1,s2,dc;} pm_ports[2];
static unsigned cfg[2][1024], initial[2][1024], regs[0x3000/4], initial_regs[0x3000/4];
static unsigned long parf;
static int writes,fail_at=-1,ignore_at=-1,training_stuck;
static int hakios_pcie_cfg_read(unsigned rc,unsigned bdf,unsigned off,unsigned size,unsigned *v) {
 unsigned i=!!bdf,shift=(off&3)*8;*v=(cfg[i][off/4]>>shift)&(size==2?65535:~0u);
 if(!bdf&&off==0x82&&training_stuck)*v|=1u<<11;
 return 0;
}
static int hakios_pcie_cfg_write(unsigned rc,unsigned bdf,unsigned off,unsigned size,unsigned v) {
 int at=writes++;if(at==fail_at)return EIO;if(at==ignore_at)return 0;
 unsigned i=!!bdf,mask=size==2?65535:~0u,shift=(off&3)*8;
 if(off==pm_ports[i].ss+8 && (v&15)) {
  assert(!(cfg[0][0x80/4]&2)&&!(cfg[1][0x80/4]&2));
  if(i)assert((cfg[0][pm_ports[0].ss/4+2]&15)==(v&15));
 }
 if(!bdf&&off==0x80)v&=~(1u<<5); /* retrain self-clears */
 cfg[i][off/4]=(cfg[i][off/4]&~(mask<<shift))|((v&mask)<<shift);return 0;
}
static int rd(unsigned long off,unsigned *v){*v=regs[off/4];return 0;}
static int wr(unsigned long off,unsigned v){if(writes++==fail_at)return EIO;regs[off/4]=v;return 0;}
static void delay(unsigned us){}
#include "hakios.pcie-pm.h"
static void reset(void) {
 memset(cfg,0,sizeof(cfg));memset(regs,0,sizeof(regs));writes=0;fail_at=ignore_at=-1;training_stuck=0;
 pm_active=pm_error=0;pm_owned=false;pm_valid=true;pm_prepared=0;
 pm_ports[0]=(struct pm_port){0,0x70,0x1fc,0x7a4c13,8,0xb012,0x1c1f,0,0x461f,0x4600,0x28};
 pm_ports[1]=(struct pm_port){0x100,0x70,0x1ec,0x424c13,0,0x1012,0x181f,0,0x461f,0,0x28};
 pm_ports[1].dc=7u<<9;
 for(unsigned i=0;i<2;i++) {
  cfg[i][0x80/4]=pm_ports[i].ctl|(pm_ports[i].sta<<16);
  cfg[i][pm_ports[i].ss/4+2]=pm_ports[i].s1;
  cfg[i][pm_ports[i].ss/4+3]=pm_ports[i].s2;
 }
 regs[0]=0x108;regs[0x2b0/4]=0xa;
 memcpy(initial,cfg,sizeof(cfg));memcpy(initial_regs,regs,sizeof(regs));
}
'''
tests=r'''
int main(void) {
 reset();assert(!pm_prepare(0)&&!pm_activate(0)&&writes==0);
 for(unsigned mode=1;mode<=3;mode++) {
  reset();assert(!pm_prepare(mode));assert(!(cfg[0][0x80/4]&3)&&!(cfg[1][0x80/4]&3));
  assert(!pm_activate(mode)&&pm_active==mode);
  assert(cfg[0][0x70c/4]&(1u<<30)); /* L1 direct from idle L0, without L0s */
  assert((cfg[0][0x80/4]&3)==2&&(cfg[1][0x80/4]&3)==2); /* never L0s */
  if(mode>=2){assert((cfg[0][pm_ports[0].ss/4+2]&15)==(mode==3?12:8));assert((cfg[1][pm_ports[1].ss/4+2]&0xe3ff0000)==0x40960000);}
  assert(!pm_restore()&&!pm_owned&&!pm_active);assert(!memcmp(initial,cfg,sizeof(cfg)));assert(!memcmp(initial_regs,regs,sizeof(regs)));
 }
 for(int n=0;n<65;n++) {
  reset();fail_at=n;int e=pm_prepare(3);if(!e)e=pm_activate(3);fail_at=-1;
  assert(!pm_restore()&&!pm_owned);assert(!memcmp(initial,cfg,sizeof(cfg)));assert(!memcmp(initial_regs,regs,sizeof(regs)));
 }
 reset();pm_ports[1].sc&=~4u;assert(pm_prepare(3)==ENOTSUP&&writes==0);
 reset();pm_ports[1].dc2&=~(1u<<11);assert(pm_prepare(3)==ENOTSUP&&writes==0);
 reset();pm_ports[1].dc=3u<<9;assert(pm_prepare(1)==ENOTSUP&&writes==0);
 reset();assert(!pm_prepare(3));pm_ports[0].sc|=3u<<16;assert(pm_activate(3)==ENOTSUP);assert(!pm_restore());
 reset();assert(!pm_prepare(3)&&!pm_activate(3));cfg[0][0x80/4]|=0x80;regs[0]|=0x8000;assert(!pm_restore());assert(cfg[0][0x80/4]&0x80);assert(regs[0]&0x8000);
 reset();training_stuck=1;assert(pm_prepare(1)==ETIMEDOUT);assert(pm_restore()==ETIMEDOUT&&pm_owned);
 reset();assert(!pm_prepare(3)&&!pm_activate(3));int before=writes;fail_at=before;
 assert(pm_restore()==EIO&&pm_owned&&writes==before+1);fail_at=-1;assert(!pm_restore());assert(!memcmp(initial,cfg,sizeof(cfg)));
 reset();ignore_at=4;assert(pm_prepare(3)==EIO&&pm_owned);ignore_at=-1;assert(!pm_restore());assert(!memcmp(initial,cfg,sizeof(cfg)));
 puts("pcie-aspm: L1/L1.1/L1.2 ordering/timing, exact restore, 65 write-failure positions, capability/scale rejection, shared-bit preservation and retrain timeout passed");
}
'''
with tempfile.TemporaryDirectory(prefix='pcie-aspm-') as tmp:
 p=Path(tmp);(p/'test.c').write_text(prefix+tests)
 subprocess.run(['clang','-std=gnu11','-O1','-g','-fsanitize=address,undefined','-I'+str(r/'drivers/wifi'),str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
