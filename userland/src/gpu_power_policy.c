/* Phone-local opt-in policy for the already tested A730 runtime controls.
 * Startup applies once. Root can persistently enable/disable each feature. */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int snapshot(char *out, size_t size) {
    size_t n=size-1;
    if (sysctlbyname("kern.hakios.gpu_power",out,&n,NULL,0)) return -1;
    out[n]=0;
    return 0;
}
static int field(const char *text,const char *key) {
    char pattern[80]; snprintf(pattern,sizeof(pattern)," %s=",key);
    const char *p=strstr(text,pattern);
    return p ? (int)strtol(p+strlen(pattern),NULL,10) : -1;
}
static int set(const char *feature,int value) {
    char name[128]; snprintf(name,sizeof(name),"kern.hakios.gpu_%s",feature);
    if (sysctlbyname(name,NULL,NULL,&value,sizeof(value))) { perror(name); return -1; }
    return 0;
}
static void marker(char *out,size_t size,const char *feature) {
    snprintf(out,size,"/private/var/db/hakios-gpu-%s-disabled",feature);
}
static int preference(const char *feature) {
    char path[160]; struct stat st; marker(path,sizeof(path),feature);
    if (lstat(path,&st)) return errno==ENOENT ? 1 : -1;
    if (!S_ISREG(st.st_mode) || st.st_uid!=0 || (st.st_mode&022)) return -1;
    return 0;
}
int main(int argc,char **argv) {
    setvbuf(stdout,NULL,_IONBF,0);
    if (geteuid()!=0) { fputs("GPU policy requires root\n",stderr); return 1; }
    if (argc==3 && (!strcmp(argv[1],"enable") || !strcmp(argv[1],"disable"))) {
        const char *feature=argv[2]; int enabled=!strcmp(argv[1],"enable");
        if (strcmp(feature,"dcvs") && strcmp(feature,"ifpc") && strcmp(feature,"slumber")) return 2;
        char path[160]; marker(path,sizeof(path),feature);
        if (!enabled) {
            int fd=open(path,O_WRONLY|O_CREAT|O_TRUNC|O_NOFOLLOW,0600);
            if (fd<0) { perror(path); return 1; }
            if (write(fd,"disabled\n",9)!=9 || fsync(fd)) { close(fd); return 1; }
            close(fd);
        }
        if (set(feature,enabled)) return 1;
        if (enabled && unlink(path) && errno!=ENOENT) { perror(path); return 1; }
        printf("GPU %s %s; preference persists across restarts\n",feature,enabled?"enabled":"disabled");
        return 0;
    }
    if (argc!=1) return 2;
    int dcvs=preference("dcvs"),ifpc=preference("ifpc"),slumber=preference("slumber");
    if (dcvs<0 || ifpc<0 || slumber<0) { fputs("Untrusted GPU policy preference\n",stderr); return 1; }
    char before[4096]={0},after[4096]={0};
    int ready=0;
    for (int i=0;i<30;i++) {
        if (!snapshot(before,sizeof(before)) && (field(before,"state")==1 || field(before,"state")==6)) { ready=1; break; }
        sleep(1);
    }
    if (!ready || field(before,"control")!=1 || field(before,"ifpc_config")!=1 || field(before,"slumber_config")!=1 ||
        field(before,"dcvs_failed")!=0 || field(before,"ifpc_failed")!=0 || field(before,"slumber_failed")!=0) {
        fprintf(stderr,"GPU policy unavailable: %s\n",before); return 1;
    }
    /* This unit/counter configuration passed idle/load validation in repeated RAM
     * boots. Require valid fresh intervals here; invalid samples keep the
     * governor's existing activity/fence fallback instead of inventing load. */
    if (set("slumber",0) || set("ifpc",ifpc)) return 1;
    int valid=0;
    for (int i=0;i<20;i++) {
        usleep(100000);
        if (!snapshot(after,sizeof(after)) && field(after,"busy_valid")==1 && field(after,"busy_rejected")==0) valid++;
        else valid=0;
        if (valid>=2) break;
    }
    int busy=valid>=2;
    if (set("busy_use",busy) || set("dcvs",dcvs) || set("slumber",slumber)) return 1;
    if (snapshot(after,sizeof(after))) return 1;
    printf("GPU policy applied: dcvs=%d busy_use=%d ifpc=%d slumber=%d\n%s\n",dcvs,busy,ifpc,slumber,after);
    return 0;
}
