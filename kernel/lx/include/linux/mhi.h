/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 * C5: public API subset of Linux v7.0.14 include/linux/mhi.h.
 * Config/client declarations retain upstream field types and enum values;
 * controller contains only public fields used by ath11k and our IPCR client.
 * Host internals live behind private_data, not Linux host locks/work queues.
 * See wifi/WAVE0.md for source hashes, callers and ownership rules.
 * This is declarations only: WP4 supplies bodies in lx_mhi.c.
 */
#ifndef _LX_LINUX_MHI_H_
#define _LX_LINUX_MHI_H_
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/types.h>
#include <linux/bitops.h>

struct mhi_controller;
struct mhi_device;
struct mhi_chan;
struct mhi_device_id;
struct module;
struct sk_buff;

enum mhi_callback {
	MHI_CB_IDLE,
	MHI_CB_PENDING_DATA,
	MHI_CB_LPM_ENTER,
	MHI_CB_LPM_EXIT,
	MHI_CB_EE_RDDM,
	MHI_CB_EE_MISSION_MODE,
	MHI_CB_SYS_ERROR,
	MHI_CB_FATAL_ERROR,
	MHI_CB_BW_REQ,
};

enum mhi_flags {
	MHI_EOB = BIT(0),
	MHI_EOT = BIT(1),
	MHI_CHAIN = BIT(2),
};

enum mhi_device_type {
	MHI_DEVICE_XFER,
	MHI_DEVICE_CONTROLLER,
};

enum mhi_ch_type {
	MHI_CH_TYPE_INVALID = 0,
	MHI_CH_TYPE_OUTBOUND = DMA_TO_DEVICE,
	MHI_CH_TYPE_INBOUND = DMA_FROM_DEVICE,
	MHI_CH_TYPE_INBOUND_COALESCED = 3,
};

enum mhi_ee_type {
	MHI_EE_PBL,
	MHI_EE_SBL,
	MHI_EE_AMSS,
	MHI_EE_RDDM,
	MHI_EE_WFW,
	MHI_EE_PTHRU,
	MHI_EE_EDL,
	MHI_EE_FP,
	MHI_EE_MAX_SUPPORTED = MHI_EE_FP,
	MHI_EE_DISABLE_TRANSITION, /* local EE, not related to mhi spec */
	MHI_EE_NOT_SUPPORTED,
	MHI_EE_MAX,
};

enum mhi_state {
	MHI_STATE_RESET = 0x0,
	MHI_STATE_READY = 0x1,
	MHI_STATE_M0 = 0x2,
	MHI_STATE_M1 = 0x3,
	MHI_STATE_M2 = 0x4,
	MHI_STATE_M3 = 0x5,
	MHI_STATE_M3_FAST = 0x6,
	MHI_STATE_BHI = 0x7,
	MHI_STATE_SYS_ERR = 0xFF,
	MHI_STATE_MAX,
};

enum mhi_er_data_type {
	MHI_ER_DATA,
	MHI_ER_CTRL,
};

enum mhi_db_brst_mode {
	MHI_DB_BRST_DISABLE = 0x2,
	MHI_DB_BRST_ENABLE = 0x3,
};

struct mhi_channel_config {
	char *name;
	u32 num;
	u32 num_elements;
	u32 local_elements;
	u32 event_ring;
	enum dma_data_direction dir;
	enum mhi_ch_type type;
	u32 ee_mask;
	u32 pollcfg;
	enum mhi_db_brst_mode doorbell;
	bool lpm_notify;
	bool offload_channel;
	bool doorbell_mode_switch;
	bool wake_capable;
};

struct mhi_event_config {
	u32 num_elements;
	u32 irq_moderation_ms;
	u32 irq;
	u32 channel;
	u32 priority;
	enum mhi_db_brst_mode mode;
	enum mhi_er_data_type data_type;
	bool hardware_event;
	bool client_managed;
	bool offload_channel;
};

struct mhi_controller_config {
	u32 max_channels;
	u32 timeout_ms;
	u32 ready_timeout_ms;
	u32 buf_len;
	u32 num_channels;
	const struct mhi_channel_config *ch_cfg;
	u32 num_events;
	struct mhi_event_config *event_cfg;
	bool use_bounce_buf;
	bool m2_no_db;
};

struct mhi_buf {
	void *buf;
	const char *name;
	dma_addr_t dma_addr;
	size_t len;
};

struct image_info {
	struct mhi_buf *mhi_buf;
	/* private: from internal.h */
	struct bhi_vec_entry *bhi_vec;
	/* public: */
	u32 entries;
};

/* Caller fills these before register; register snapshots config, not firmware.
 * irq[] is caller-owned through unregister, freed by ath11k afterward.
 * iova_stop is inclusive, matching Linux. WP4 must intersect this with C4's
 * pool: ath11k sets the whole DMA mask, not the allocated DMA window.
 * fbc_image/rddm_image remain NULL until populated; RDDM is unsupported.
 */
struct mhi_controller {
    struct device *cntrl_dev;
    struct mhi_device *mhi_dev;
    void __iomem *regs;
    size_t reg_len;
    dma_addr_t iova_start, iova_stop;
    const char *fw_image;
    const u8 *fw_data;
    size_t fw_sz, rddm_size, sbl_size, seg_len;
    struct image_info *fbc_image, *rddm_image;
    int *irq;
    u32 nr_irqs, timeout_ms;
    size_t buffer_len;
    bool fbc_download;
    unsigned long irq_flags;
    int (*runtime_get)(struct mhi_controller *mhi_cntrl);
    void (*runtime_put)(struct mhi_controller *mhi_cntrl);
    void (*status_cb)(struct mhi_controller *mhi_cntrl, enum mhi_callback cb);
    int (*read_reg)(struct mhi_controller *mhi_cntrl, void __iomem *addr, u32 *out);
    void (*write_reg)(struct mhi_controller *mhi_cntrl, void __iomem *addr, u32 val);
    void *private_data; /* ours, allocated/freed by WP4 */
};

struct mhi_device {
	const struct mhi_device_id *id;
	const char *name;
	struct mhi_controller *mhi_cntrl;
	struct mhi_chan *ul_chan;
	struct mhi_chan *dl_chan;
	struct device dev;
	enum mhi_device_type dev_type;
	int ul_chan_id;
	int dl_chan_id;
	u32 dev_wake;
};

struct mhi_result {
	void *buf_addr;
	size_t bytes_xferd;
	enum dma_data_direction dir;
	int transaction_status;
};

struct mhi_driver {
	const struct mhi_device_id *id_table;
	int (*probe)(struct mhi_device *mhi_dev,
		     const struct mhi_device_id *id);
	void (*remove)(struct mhi_device *mhi_dev);
	void (*ul_xfer_cb)(struct mhi_device *mhi_dev,
			   struct mhi_result *result);
	void (*dl_xfer_cb)(struct mhi_device *mhi_dev,
			   struct mhi_result *result);
	void (*status_cb)(struct mhi_device *mhi_dev, enum mhi_callback mhi_cb);
	struct device_driver driver;
};

struct mhi_controller *mhi_alloc_controller(void);

void mhi_free_controller(struct mhi_controller *mhi_cntrl);

int mhi_register_controller(struct mhi_controller *mhi_cntrl,
			const struct mhi_controller_config *config);

void mhi_unregister_controller(struct mhi_controller *mhi_cntrl);

int mhi_prepare_for_power_up(struct mhi_controller *mhi_cntrl);

int mhi_async_power_up(struct mhi_controller *mhi_cntrl);

int mhi_sync_power_up(struct mhi_controller *mhi_cntrl);

void mhi_power_down(struct mhi_controller *mhi_cntrl, bool graceful);

void mhi_power_down_keep_dev(struct mhi_controller *mhi_cntrl, bool graceful);

void mhi_unprepare_after_power_down(struct mhi_controller *mhi_cntrl);

int mhi_pm_suspend(struct mhi_controller *mhi_cntrl);

int mhi_pm_resume_force(struct mhi_controller *mhi_cntrl);

int mhi_download_rddm_image(struct mhi_controller *mhi_cntrl, bool in_panic);

int mhi_device_get_sync(struct mhi_device *mhi_dev);

void mhi_device_put(struct mhi_device *mhi_dev);

int mhi_prepare_for_transfer(struct mhi_device *mhi_dev);

void mhi_unprepare_from_transfer(struct mhi_device *mhi_dev);

int mhi_queue_buf(struct mhi_device *mhi_dev, enum dma_data_direction dir,
		  void *buf, size_t len, enum mhi_flags mflags);

int mhi_queue_skb(struct mhi_device *mhi_dev, enum dma_data_direction dir,
		  struct sk_buff *skb, size_t len, enum mhi_flags mflags);

int mhi_get_free_desc_count(struct mhi_device *mhi_dev,
				enum dma_data_direction dir);

enum mhi_state mhi_get_mhi_state(struct mhi_controller *mhi_cntrl);

int __mhi_driver_register(struct mhi_driver *mhi_drv, struct module *owner);

void mhi_driver_unregister(struct mhi_driver *mhi_drv);

/* Linux semantics: int APIs return 0/-errno, except get_free_desc_count which
 * returns a count/-errno. queue success transfers buffer ownership until its
 * completion callback (including -ENOTCONN on reset); failure retains it.
 * All callbacks run in thread context, outside host locks. unprepare and
 * unregister drain callbacks before freeing device/driver storage.
 * Suspend/resume initially log no-op; RDDM returns -EOPNOTSUPP.
 */
#define mhi_driver_register(drv) __mhi_driver_register(drv, THIS_MODULE)
#define module_mhi_driver(drv) \
    module_driver(drv, mhi_driver_register, mhi_driver_unregister)
#endif
