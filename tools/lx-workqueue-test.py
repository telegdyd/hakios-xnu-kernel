"""Run production workqueue creation/drain/destruction with real host threads."""
from pathlib import Path
import subprocess,tempfile
r=Path(__file__).resolve().parents[1]
s=(r/'wifi/lx/src/lx_base.c').read_text()
s=s[s.index('enum { WORK_IDLE'):s.index('\nbool\nlinux_cancel_delayed_work_sync')]
prefix=r'''
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#define __containerof(p,t,m) ((t*)((char*)(p)-offsetof(t,m)))
#define nitems(a) (sizeof(a)/sizeof((a)[0]))
#define SLICE_NS 1000000
struct workqueue_struct { int dummy; };
struct work_struct { atomic_int state;struct workqueue_struct *work_queue;void (*func)(struct work_struct *); };
#define atomic_read(p) atomic_load(p)
#define atomic_set(p,v) atomic_store(p,v)
static pthread_mutex_t lock=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond=PTHREAD_COND_INITIALIZER;
static atomic_int outstanding;
static void lx_os_lock(void){pthread_mutex_lock(&lock);}
static void lx_os_unlock(void){pthread_mutex_unlock(&lock);}
static void lx_os_sleep(const void *p,unsigned long long ns){pthread_cond_wait(&cond,&lock);}
static void lx_os_wakeup(const void *p){pthread_cond_broadcast(&cond);}
static void *lx_os_thread_self(void){return (void*)(uintptr_t)pthread_self();}
static void *lx_os_alloc(size_t n){outstanding++;return calloc(1,n);}
static void lx_os_free(void *p,size_t n){outstanding--;free(p);}
static void lx_os_panic(const char *s){fprintf(stderr,"%s\n",s);abort();}
#define strlcpy fixture_strlcpy
static size_t strlcpy(char *d,const char *s,size_t n){snprintf(d,n,"%s",s);return strlen(s);}
struct start {void (*fn)(void*);void *arg;};
static void *entry(void *p){struct start a=*(struct start*)p;free(p);a.fn(a.arg);return NULL;}
static int lx_os_thread(const char *name,void (*fn)(void*),void *arg){pthread_t t;struct start *p=malloc(sizeof(*p));*p=(struct start){fn,arg};int e=pthread_create(&t,NULL,entry,p);if(!e)pthread_detach(t);return e;}
'''
tests=r'''
static atomic_int calls,started,release_work,destroyed;
static void callback(struct work_struct *w){calls++;}
static void held(struct work_struct *w){started=1;while(!release_work){}calls++;}
static void *destroy(void *p){linux_destroy_workqueue(p);destroyed=1;return NULL;}
int main(void){
 for(int i=0;i<100;i++){
  struct workqueue_struct *q=linux_create_workqueue_common("fixture",1);
  struct work_struct a={.func=callback},b={.func=callback};
  assert(linux_queue_work_on(0,q,&a));assert(linux_queue_work_on(0,q,&b));
  linux_destroy_workqueue(q);assert(outstanding==0);
 }
 assert(calls==200);
 struct workqueue_struct *q=linux_create_workqueue_common("held",1);
 struct work_struct a={.func=held},b={.func=callback};pthread_t t;
 assert(linux_queue_work_on(0,q,&a));while(!started){}
 assert(linux_queue_work_on(0,q,&b));pthread_create(&t,NULL,destroy,q);
 lx_os_lock();while(!((struct lx_workqueue*)q)->stopping)lx_os_sleep(q,0);lx_os_unlock();
 assert(!destroyed);release_work=1;pthread_join(t,NULL);
 assert(destroyed && calls==202 && outstanding==0);
 puts("lx-workqueue: 100 create/drain/destroy cycles, running callback barrier, queued callback drain and zero leaked queues passed");
}
'''
with tempfile.TemporaryDirectory(prefix='lx-workqueue-') as tmp:
 p=Path(tmp);(p/'test.c').write_text(prefix+s+tests)
 subprocess.run(['clang','-std=gnu11','-O1','-g','-pthread','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True,timeout=20)
