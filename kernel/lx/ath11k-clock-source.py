#!/usr/bin/env python3
"""Prepare build copies for the clock accessor and deferred monitor statistics.

The pinned upstream source remains untouched. Only a structure member's name
changes; its declaration and accesses must agree in every translation unit.
The monitor-status timer is serviced on existing PCI worker wakes. A country the host asked for is not
intersected with the board data's domain (reg.c). WMI_WOW_ENABLE's
flags come from lx_bus.c (the DRV sleep flag). Protocol
timers, including fragment expiry and connection loss, keep their deadlines.
"""
from pathlib import Path
import sys

source, output = map(Path, sys.argv[1:])
output.mkdir(parents=True, exist_ok=True)
common = source.parent / 'spectral_common.h'
common_output = output.parent / common.name
if not common_output.exists() or common_output.read_bytes() != common.read_bytes():
    common_output.write_bytes(common.read_bytes())
for path in source.iterdir():
    if not path.is_file():
        continue
    data = path.read_bytes()
    if path.name == 'core.h':
        old = b'unsigned long jiffies;'
        assert data.count(old) == 1, 'ath11k timestamp declaration changed'
        data = data.replace(old, b'unsigned long timestamp_jiffies;')
    data = data.replace(b'->jiffies', b'->timestamp_jiffies')
    if path.name == 'dp_rx.c':
        old = b'timer_setup(&ar->ab->mon_reap_timer,\n\t\t\t    ath11k_dp_service_mon_ring, 0);'
        assert data.count(old) == 1, 'ath11k monitor timer initialization changed'
        data = data.replace(old, old + b'\n\t\tlx_timer_defer(&ar->ab->mon_reap_timer);')
    if path.name == 'wmi.c':
        # WMI_WOW_ENABLE's flags from lx_bus.c (lx_wow_enable_flags: the DRV sleep flag the firmware needs to wake the
        # host from WoW when the link sleeps held by the audio DSP); upstream sends 0
        old = b'\tcmd->pause_iface_config = WOW_IFACE_PAUSE_ENABLED;\n'
        assert data.count(old) == 1, 'ath11k WoW enable command changed'
        data = data.replace(old, old + b'\t{ extern u32 lx_wow_enable_flags(void); cmd->flags = lx_wow_enable_flags(); }\n')
    if path.name == 'wow.c':
        old = b'bitmap = WMI_HW_DATA_FILTER_DROP_NON_ICMPV6_MC |\n\t\t\tWMI_HW_DATA_FILTER_DROP_NON_ARP_BC;'
        assert data.count(old) == 1, 'ath11k WoW receive filter changed'
        # Hakios PCIe attaches only the 17cb:1103 endpoint. Its firmware fails IPv6
        # neighbor discovery in WoW with the multicast filter set.
        data = data.replace(old, old + (
            b'\n\t\tif (ar->ab->hif.bus == ATH11K_BUS_PCI)'
            b'\n\t\t\tbitmap &= ~WMI_HW_DATA_FILTER_DROP_NON_ICMPV6_MC;'
        ))
    if path.name == 'reg.c':
        # A country the host asked for (ath11k_reg_set_cc() keeps it in ar->alpha2: wifid's, or the beacons' 802.11d
        # one, lx_80211.c) is the firmware's domain for that country as it is. Upstream intersects every domain after
        # the first with the board data's (US on the NE2213): Europe's channels 12 and 13 at 2.4 GHz and most of
        # 6 GHz would never be used, and a network on 13 is never heard. Other domain changes keep upstream's rule.
        old = (b'\t    !ath11k_reg_is_world_alpha((char *)reg_info->alpha2))\n'
               b'\t\tintersect = true;\n')
        assert data.count(old) == 1, 'ath11k regulatory intersection changed'
        data = data.replace(old, (b'\t    !ath11k_reg_is_world_alpha((char *)reg_info->alpha2))\n'
                                  b'\t\tintersect = memcmp(ab->pdevs[pdev_idx].ar->alpha2, reg_info->alpha2, 2) != 0;\n'))
    target = output / path.name
    if not target.exists() or target.read_bytes() != data:
        target.write_bytes(data)
