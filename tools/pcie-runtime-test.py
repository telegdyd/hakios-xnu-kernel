"""Execute production runtime PCIe transitions against a lifecycle fixture."""
from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'wifi/lx/src/lx_bus.c').read_text()
s=s[s.index('static struct ath11k_base *volatile driver;'):s.index('\nint\nlx_start(void)')]
prefix=r'''
#include <assert.h>
#include <stdbool.h>
#include <errno.h>
#include <stdio.h>
#define CONFIG_ATH11K_PCI 1
#define ATH11K_FLAG_REGISTERED 0
#define ATH11K_FLAG_QMI_FAIL 1
struct ath11k_base { unsigned long dev_flags; } chip={1};
static int prepared, live, status, prep_error, init_error, exits, leaves, fail_ready;
static unsigned long long ticks;
static void lx_os_lock(void) {}
static void lx_os_unlock(void) {}
static void lx_os_sleep(void *p,unsigned long long ns) { assert(0); }
static int lx_prepare(void) { if(prep_error)return prep_error;prepared=1;return 0; }
static int lx_os_pcie_down_status(void) { return status; }
static int lx_pci_driver_init(void) { assert(prepared);if(init_error)return init_error;live=1;chip.dev_flags=fail_ready?2:1;return 0; }
static void lx_pci_driver_exit(void) { exits++;live=0;prepared=0; }
static struct ath11k_base *lx_pci_base(void) { return live?&chip:0; }
static unsigned long long lx_os_nanoseconds(void) { ticks+=1000000;return ticks; }
static void lx_pause_ms(unsigned ms) { ticks+=ms*1000000ull; }
static bool test_bit(int bit,unsigned long *p) { return (*p>>bit)&1; }
static void lx_station_leave(void) { leaves++; }
static void lx_os_runtime_state(int p,int e) {}
static void lx_os_qrtr_diagnostic(void) {}
static void lx_mhi_diagnostic(void) {}
static int lx_os_pcie_pm_start(void) { return 0; }
#define lx_tell(...) ((void)0)
'''
tests=r'''
int main(void) {
 assert(lx_pci_runtime_power(false)==0 && exits==0);
 assert(lx_pci_runtime_power(true)==0 && driver==&chip && prepared);
 assert(lx_pci_runtime_power(true)==0 && exits==0);
 assert(lx_pci_runtime_power(false)==0 && !driver && exits==1 && leaves==1 && !prepared);
 assert(lx_pci_runtime_power(true)==0 && driver==&chip); /* must reacquire pool */
 assert(lx_pci_runtime_power(false)==0 && exits==2);
 status=EIO; assert(lx_pci_runtime_power(true)==EIO && !driver && !prepared);
 assert(lx_pci_runtime_power(false)==EIO); status=0;
 prep_error=ENOMEM; assert(lx_pci_runtime_power(true)==ENOMEM && !driver);prep_error=0;
 init_error=-ENODEV; assert(lx_pci_runtime_power(true)==ENODEV && !driver);init_error=0;
 fail_ready=1;assert(lx_pci_runtime_power(true)==EIO && !driver && !live && exits==3);
 puts("pcie-runtime: off/on, duplicate requests, pool reacquisition, uncertain shutdown, allocation/probe/firmware failures passed");
}
'''
with tempfile.TemporaryDirectory(prefix='pcie-runtime-') as tmp:
 p=Path(tmp);(p/'test.c').write_text(prefix+s+tests)
 subprocess.run(['clang','-std=gnu11','-O1','-g','-fsanitize=address,undefined','-Werror','-Wno-incompatible-pointer-types-discards-qualifiers',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
