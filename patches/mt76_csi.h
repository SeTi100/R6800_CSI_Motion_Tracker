/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * MT76 CSI (Channel State Information) Extraction
 * For MT7615E on Netgear R6800 - Micro-Doppler Wi-Fi Sensing
 *
 * Copyright (C) 2026 R6800 CSI Project
 */

#ifndef __MT76_CSI_H
#define __MT76_CSI_H

#include <linux/version.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

/* CSI Configuration */
#define MT76_CSI_MAX_SUBCARRIERS  64   /* 56 usable + 8 null for 20MHz HT */
#define MT76_CSI_MAX_ANTENNAS     4    /* MT7615 supports up to 4x4 */
#define MT76_CSI_BUF_COUNT        256  /* Ring buffer entries */

/* CSI Data Mode */
enum mt76_csi_mode {
	MT76_CSI_MODE_DISABLED = 0,
	MT76_CSI_MODE_IQ_RAW,          /* Full I/Q per subcarrier */
	MT76_CSI_MODE_AMPLITUDE_ONLY,  /* Amplitude only */
	MT76_CSI_MODE_METRICS,         /* Aggregated: RSSI, SNR, etc */
};

/*
 * struct mt76_csi_data - Single CSI measurement record
 *
 * Layout optimized for UDP transport (~480 bytes per entry
 * with 2 antennas x 56 subcarriers)
 */
struct mt76_csi_data {
	/* Timing */
	u64 timestamp_us;              /* ktime_get_ns() / 1000 */
	u32 seq_num;                   /* Monotone sequence number */
	u16 frame_seq;                 /* 802.11 Sequence Number */

	/* Wireless Metadata */
	u8  band;                      /* 0=2.4GHz, 1=5GHz */
	u8  bw;                        /* 0=20MHz, 1=40MHz, 2=80MHz */
	u8  channel;                   /* Channel number */
	u8  n_rx;                      /* Number of Rx antennas used */
	u8  n_tx;                      /* Number of detected Tx antennas */
	u8  n_subcarriers;             /* Number of subcarriers reported */
	s8  rssi[MT76_CSI_MAX_ANTENNAS]; /* Per-antenna RSSI (dBm) */
	u8  noise_floor;               /* Noise floor (dBm) */
	u8  _pad0;

	/* Source identification */
	u8  src_mac[6];                /* Sender MAC address */
	s16 foe;                       /* Frequency Offset Estimation */

	/* I/Q Data: [antenna][subcarrier]
	 * For 2x2 @ 20MHz: 2 * 56 * 2 * 2 = 448 bytes
	 */
	s16 i_data[MT76_CSI_MAX_ANTENNAS][MT76_CSI_MAX_SUBCARRIERS];
	s16 q_data[MT76_CSI_MAX_ANTENNAS][MT76_CSI_MAX_SUBCARRIERS];
} __packed;

/*
 * struct mt76_csi_buf - CSI ring buffer with configuration
 */
struct mt76_csi_buf {
	struct mt76_csi_data *entries;  /* Dynamically allocated */
	u32 size;                      /* Buffer size (power of 2) */
	u32 head;                      /* Write index (kernel) */
	u32 tail;                      /* Read index (userspace) */
	u32 overflow_count;            /* Overflow counter */
	spinlock_t lock;

	/* Configuration */
	enum mt76_csi_mode mode;
	u8  filter_mac[6];             /* MAC filter (optional) */
	bool filter_enabled;
	bool capture_active;
	u32 total_captured;
	u32 total_dropped;

	/* Waitqueue for poll/select */
	wait_queue_head_t wait;
};

/* Function prototypes */
struct mt76_csi_buf *mt76_csi_buf_alloc(u32 size);
void mt76_csi_buf_free(struct mt76_csi_buf *buf);
struct mt76_csi_data *mt76_csi_buf_write_begin(struct mt76_csi_buf *buf);
void mt76_csi_buf_write_end(struct mt76_csi_buf *buf);
int mt76_csi_buf_read(struct mt76_csi_buf *buf, struct mt76_csi_data *out,
		      int max_entries);

#endif /* __MT76_CSI_H */
