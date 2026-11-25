/* The fake kernel's <net/kpi_interface.h>: the checksum offload flags board/net.h names (their values in XNU's). */
#ifndef FAKE_NET_KPI_INTERFACE_H
#define FAKE_NET_KPI_INTERFACE_H

#define IFNET_CSUM_IP           0x00000001
#define IFNET_CSUM_TCP          0x00000002
#define IFNET_CSUM_UDP          0x00000004
#define IFNET_CSUM_TCPIPV6      0x00000020
#define IFNET_CSUM_UDPIPV6      0x00000040

#endif

#ifndef FAKE_NET_KPI_INTERFACE_IFNET
#define FAKE_NET_KPI_INTERFACE_IFNET
#include <errno.h>
#include <sys/socket.h>
/* interfaces and their addresses: there are none here (board/lx.c's wake patterns then have no address to offload) */
typedef struct fake_ifnet *ifnet_t;
typedef struct fake_ifaddr *ifaddr_t;
static inline int ifnet_find_by_name(const char *name, ifnet_t *ifp) { (void)name; *ifp = NULL; return ENXIO; }
static inline int ifnet_get_address_list_family(ifnet_t ifp, ifaddr_t **list, sa_family_t family)
{ (void)ifp; (void)family; *list = NULL; return ENXIO; }
static inline int ifaddr_address(ifaddr_t ifa, struct sockaddr *out, unsigned size) { (void)ifa; (void)out; (void)size; return EINVAL; }
static inline void ifnet_free_address_list(ifaddr_t *list) { (void)list; }
static inline int ifnet_release(ifnet_t ifp) { (void)ifp; return 0; }
#endif
