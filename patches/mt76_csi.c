// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * MT76 CSI Ring Buffer Implementation
 */

#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/log2.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/export.h>
#include <linux/vmalloc.h>
#include "mt76_csi.h"

struct mt76_csi_buf *mt76_csi_buf_alloc(u32 size)
{
	struct mt76_csi_buf *buf;

	/* Round up to power of 2 for efficient masking */
	size = roundup_pow_of_two(size);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return NULL;

	buf->entries = vzalloc(size * sizeof(struct mt76_csi_data));
	if (!buf->entries) {
		kfree(buf);
		return NULL;
	}

	buf->size = size;
	buf->head = 0;
	buf->tail = 0;
	buf->overflow_count = 0;
	buf->total_captured = 0;
	buf->total_dropped = 0;
	buf->mode = MT76_CSI_MODE_IQ_RAW;
	buf->capture_active = false;
	buf->filter_enabled = false;
	spin_lock_init(&buf->lock);
	init_waitqueue_head(&buf->wait);

	return buf;
}

void mt76_csi_buf_free(struct mt76_csi_buf *buf)
{
	if (!buf)
		return;
	vfree(buf->entries);
	kfree(buf);
}

/*
 * Get a write slot in the ring buffer.
 * Caller must hold buf->lock.
 * Returns pointer to entry to fill, or NULL if disabled.
 */
struct mt76_csi_data *mt76_csi_buf_write_begin(struct mt76_csi_buf *buf)
{
	u32 idx;

	if (!buf || !buf->capture_active)
		return NULL;

	idx = buf->head & (buf->size - 1);
	buf->entries[idx].seq_num = buf->head + 1;
	return &buf->entries[idx];
}

/*
 * Commit the written entry and advance head.
 * Caller must hold buf->lock.
 */
void mt76_csi_buf_write_end(struct mt76_csi_buf *buf)
{
	buf->head++;
	buf->total_captured++;

	/* Detect overflow: drop oldest entries */
	if (buf->head - buf->tail > buf->size) {
		buf->overflow_count++;
		buf->total_dropped += (buf->head - buf->tail) - buf->size;
		buf->tail = buf->head - buf->size;
	}

	/* Wake up any readers waiting for data */
	wake_up_interruptible(&buf->wait);
}

/*
 * Read up to max_entries from the ring buffer.
 * Returns number of entries read.
 */
int mt76_csi_buf_read(struct mt76_csi_buf *buf, struct mt76_csi_data *out,
		      int max_entries)
{
	unsigned long flags;
	int count = 0;
	u32 idx;

	if (!buf || !out || max_entries <= 0)
		return 0;

	spin_lock_irqsave(&buf->lock, flags);

	while (count < max_entries && buf->tail != buf->head) {
		idx = buf->tail & (buf->size - 1);
		memcpy(&out[count], &buf->entries[idx],
		       sizeof(struct mt76_csi_data));
		buf->tail++;
		count++;
	}

	spin_unlock_irqrestore(&buf->lock, flags);

	return count;
}
EXPORT_SYMBOL_GPL(mt76_csi_buf_alloc);
EXPORT_SYMBOL_GPL(mt76_csi_buf_free);
EXPORT_SYMBOL_GPL(mt76_csi_buf_write_begin);
EXPORT_SYMBOL_GPL(mt76_csi_buf_write_end);
EXPORT_SYMBOL_GPL(mt76_csi_buf_read);
