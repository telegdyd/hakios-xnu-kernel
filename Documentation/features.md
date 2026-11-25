# What each feature needs

A board lists the drivers it may start in `features` of its `board.dts`. Each needs to be told
where things are first: sections of the SoC file (shared by every phone with that SoC) and, for a
part on the board, a section of the board file. `tools/boards.py check` refuses a board that lists
a feature without them, and `tools/boards.py status NAME` says the same for one phone.

This page is written by `tools/boards.py features` from the tables the checker uses; a test keeps
the two the same. What each key means is in `arch/arm64/boot/dts/_template/soc.dtsi` and
`arch/arm64/boot/dts/_template/board.dts`.

| Feature | SoC file | Board file | Started with |
|---|---|---|---|
| `cpufreq-epss` | `cpufreq { }` | nothing | `cpufreq=1` |
| `ddr-rpmh` | `rpmh { }` | nothing | `ddr=1 looks; ddr=2 with cpufreq=1 votes` |
| `battery-uefi` | nothing | nothing | `battery=1` |
| `buttons-pmic` | `spmi { }` | `buttons { }`: `pon`, `volume-up` | `buttons=1` |
| `adsp` | `rpmh { }`; `ipc { }` | `adsp { }`: `firmware`, `memory`, `memory-size` | `adsp=1` |
| `touch-goodix` | `pins { }`; `clocks { }`; `rpmh { }` | `touch { }`: `serial-engine`, `clocks`, `vote-register`, `votes`, `bus-pins`, `interrupt`, `reset`, `io-supply`, `analog-enable`, `vendor-pins`, `coordinates` | `touchscreen=1` |
| `touch-ft3658u` | `pins { }`; `clocks { }`; `rpmh { }` | `touch { }`: `serial-engine`, `clocks`, `vote-register`, `votes`, `bus-pins`, `interrupt`, `reset`, `io-supply`, `analog-supply` | `touchscreen=1` |
| `speaker-aw88261` | `pins { }`; `clocks { }`; `rpmh { }`; `smmu { }`: `audio-stream` | `speaker { }`: `sda`, `scl`, `addresses`, `interrupts`, `sound-pins`, `sound-function` | `speaker_probe=N looks; sound=N with adsp=1` |
| `haptic-aw86927` | `pins { }` | `haptic { }`: `sda`, `scl`, `reset`, `interrupt`, `f0-calibration` | `haptic=N` |
| `torch-pm8350c` | `spmi { }`: `channels` | `torch { }`: `flash`, `alarm` | `torch=1 looks; torch=3 is the light for Darwin` |
| `camera-sm7325` | `spmi { }`: `channels`; `pins { }`; `clocks { }`; `rpmh { }`; `smmu { }`: `camera-stream`; `camera { }` | `camera { }`: `pmic-pins`, `pmic-reset`, `sensors` | `cam=N` |
| `wifi-wcn6750` | `rpmh { }`; `pins { }`; `spmi { }`; `ipc { }`; `smmu { }`: `wifi-stream` | `wifi { }`: `radio-model`, `dsp-memory`, `dsp-memory-size` | `wifi=1` |
| `mic-wcd938x` | `pins { }`; `lpass { }` | `mic { }`: `codec-reset` | `mic=N with adsp=1` |
| `display-scan` | `display { }`: `intf1`; `clocks { }`: `gcc-disp-ahb-cbcr`, `dispcc`, `dispcc-mdss-gdscr`, `dispcc-mdss-ahb-cbcr` | nothing | `fb_scan=1, the default` |
| `gpu-a660` | `adreno { }`; `clocks { }`: `gcc-disp-ahb-cbcr`; `rpmh { }` | `gpu { }`: `model`, `chip-id`, `firmware`, `gmem-kb`, `primitive-fifo`, `hang-cycles`, `speed-fuse`, `gaming-fuse`, `highest-bank-bit`, `gcc-debug-gpucc`, `speeds-khz`, `speed-levels`, `default-speed` | `gpu=1` |
| `virtio` | `virtio { }` | nothing | `virtio_net=1, virtio_input=1` |
| `cpufreq-scmi` | `scmi-perf { }` | nothing | `scmi_perf=1` |
| `bluetooth-wcn-uart` | `pins { }`; `clocks { }`; `rpmh { }`; `spmi { }`: `channels` | `bluetooth { }`: `serial-engine`, `wrapper`, `clocks`, `clock-source`, `vote-register`, `votes`, `uart-pins`, `uart-function`, `enable-ppid`, `sw-ctrl`, `supplies`, `supply-mv`, `rf-clock` | `bt=-1 looks; bt=1 powers and identifies; bt=2 then hands it to Darwin` |
| `modem-sm7325` | nothing | nothing | `modem=1` |
| `imu-lsm6dso` | `pins { }` | `imu { }`: `sda`, `scl`, `interrupt`, `address`, `full-scale`, `mount` | `sensors=1 looks; sensors=2 follows the orientation` |
| `imu-bmi160` | `pins { }` | `imu { }`: `sda`, `scl`, `interrupt`, `address`, `full-scale`, `mount` | `sensors=1 looks; sensors=2 follows the orientation` |
| `touch-ft5x06` | `pins { }` | `touch-i2c { }`: `sda`, `scl`, `interrupt`, `reset`, `address`, `contacts`, `coordinates` | `touchscreen=-1 looks; touchscreen=1` |
| `touch-s3908` | `pins { }` | `touch-i2c { }`: `sda`, `scl`, `interrupt`, `reset`, `address`, `contacts`, `coordinates` | `touchscreen=-1 looks; touchscreen=1` |
| `gpu-a730` | `adreno { }`; `clocks { }`: `gcc-disp-ahb-cbcr`; `rpmh { }` | `gpu { }`: `model`, `chip-id`, `firmware`, `gmem-kb`, `primitive-fifo`, `hang-cycles`, `speed-fuse`, `gaming-fuse`, `highest-bank-bit`, `gcc-debug-gpucc`, `speeds-khz`, `speed-levels`, `default-speed` | `gpu=1 or adreno=22; gpu_irq=1 for completion interrupts` |
| `display-flip` | `display { }`: `intf1`, `mdss` | `display { }`: `splash-size` | `fb_flip=1, the default, with the GPU putting frames on the panel (gpu=1; fb_gpu=1, the default)` |
| `gpu-dcvs` | `adreno { }`: `tsens` | `gpu { }`: `model`, `chip-id`, `firmware`, `gmem-kb`, `primitive-fifo`, `hang-cycles`, `speed-fuse`, `gaming-fuse`, `highest-bank-bit`, `gcc-debug-gpucc`, `speeds-khz`, `speed-levels`, `default-speed` | `gpu=1, on by itself; gpu_dcvs=0 keeps the default speed (the GPU's sensors still cap it)` |
| `gpu-ifpc` | `adreno { }` | `gpu { }`: `model`, `chip-id`, `firmware`, `gmem-kb`, `primitive-fifo`, `hang-cycles`, `speed-fuse`, `gaming-fuse`, `highest-bank-bit`, `gcc-debug-gpucc`, `speeds-khz`, `speed-levels`, `default-speed` | `gpu=1, on by itself; gpu_ifpc=0 keeps the core up between frames` |
| `gpu-slumber` | `adreno { }` | `gpu { }`: `model`, `chip-id`, `firmware`, `gmem-kb`, `primitive-fifo`, `hang-cycles`, `speed-fuse`, `gaming-fuse`, `highest-bank-bit`, `gcc-debug-gpucc`, `speeds-khz`, `speed-levels`, `default-speed` | `gpu=1, on by itself; gpu_slumber=0 keeps the GPU awake` |
| `display-power` | `display { }`: `intf1`, `mdss`, `vbif`; `clocks { }`: `gcc-disp-ahb-cbcr`, `dispcc`, `dispcc-mdss-gdscr`, `dispcc-mdss-ahb-cbcr` | `display { }`: `splash-size` | `backlight=2, on by itself; display_power_down=0 keeps the display hardware on while the panel sleeps` |
| `cpu-idle` | `cpu-idle { }` | nothing | `on by itself; cpu_idle=0 keeps idle cores in WFI, screen_off_cores=0 every core up with the display off` |

The brightness (`backlight { }`), the thermal floors (`thermal { }`), the PMIC's clock (`rtc { }`, which
needs `spmi { }` in the SoC file) and the partition that may be written (`storage { }`) are not
features: a board has them by giving the section. `display-scan` is for a command-mode panel only.
`touch-ft5x06` needs `rpmh { }` in the SoC file as well when its `touch-i2c { }` names a regulator
(`io-supply`, `analog-supply` with `analog-millivolts`): a supply that is always on is not named.

"nothing" is what the checker asks for, not what the driver needs of the phone. `modem-sm7325` has
where an SM7325 keeps its shared memory, its command database and the modem's memory in its code
(`drivers/board/mpss.c`): it is for a phone on that chip with the same memory map. `battery-uefi`
shows what the loader read from the firmware: with the loader here, `uefi-charger` in `loader { }`
of the board file.
