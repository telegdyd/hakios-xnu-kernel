#!/usr/bin/env python3
"""Prepare build copies for the clock accessor and deferred monitor statistics.

The pinned upstream source remains untouched. Only a structure member's name
changes; its declaration and accesses must agree in every translation unit.
The monitor-status timer is serviced on existing PCI worker wakes. WMI_WOW_ENABLE's
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
    target = output / path.name
    if not target.exists() or target.read_bytes() != data:
        target.write_bytes(data)
