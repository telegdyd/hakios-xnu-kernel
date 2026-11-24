"""Exercise the production LinuxKPI thread registry across repeated lifetimes."""
from pathlib import Path
import subprocess,tempfile
r=Path(__file__).resolve().parents[1]
s=(r/'wifi/lx/src/lx_base.c').read_text()
s=s[s.index('static struct lx_thread {'):s.index('\nstruct thread *\nlx_curthread')]
k=(r/'drivers/wifi/hakios.lx.c').read_text()
k=k[k.index('static void\nlx_thread_runs'):k.index('\nint\nlx_os_thread(')]
k=k.replace('struct lx_thread','struct start_data')
prefix=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define nitems(a) (sizeof(a)/sizeof((a)[0]))
struct thread { int td_tid; void *task; };
struct task_struct { unsigned state; };
static uintptr_t self;
static unsigned used, high;
static void *lx_os_thread_self(void) { return (void*)self; }
static void lx_os_thread_slots(unsigned u,unsigned h) { used=u; high=h; }
__attribute__((noreturn)) static void lx_os_panic(const char *s) { puts(s); abort(); }
struct start_data { void (*function)(void*); void *argument; };
typedef int wait_result_t;
#define __unused __attribute__((unused))
#define kfree_type(t,p) free(p)
#define current_thread() ((void*)self)
static void thread_terminate(void *t) {}
'''
tests=r'''
static void driver_function(void *p) { lx_thread()->deadline=99; }
int main(void) {
 for (unsigned round=0;round<1000;round++) {
  for (unsigned i=0;i<24;i++) {
   self=(uintptr_t)round*24+i+1;
   struct lx_thread *t=lx_thread();
   assert(t==lx_thread());
   assert(!t->task.state && !t->waiting_on && !t->deadline);
   t->task.state=123;t->waiting_on=(void*)1;t->deadline=456;
  }
  assert(used==24 && high==24);
  for (unsigned i=0;i<24;i++) {
   self=(uintptr_t)round*24+i+1;
   lx_thread_release();lx_thread_release(); /* harmless when no slot is owned */
  }
  assert(!used);
 }
 for (unsigned i=0;i<1000;i++) {
  self=100000+i;
  struct start_data *p=malloc(sizeof(*p));
  *p=(struct start_data){driver_function,NULL};
  lx_thread_runs(p,0);
  assert(!used);
 }
 puts("lx-thread: 24000 distinct thread lifetimes, full-capacity reuse, cleared state, duplicate release and 1000 production C4 thread exits passed");
}
'''
with tempfile.TemporaryDirectory(prefix='lx-thread-') as tmp:
 p=Path(tmp);(p/'test.c').write_text(prefix+s+k+tests)
 subprocess.run(['clang','-std=gnu11','-O1','-g','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
