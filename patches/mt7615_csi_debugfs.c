// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * MT7615 CSI debugfs interface
 *
 * Provides control and status via:
 *   /sys/kernel/debug/ieee80211/phyX/mt76/csi_enable
 *   /sys/kernel/debug/ieee80211/phyX/mt76/csi_mode
 *   /sys/kernel/debug/ieee80211/phyX/mt76/csi_stats
 *   /sys/kernel/debug/ieee80211/phyX/mt76/csi_filter_mac
 *   /sys/kernel/debug/ieee80211/phyX/mt76/csi_data
 */

#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include "mt7615.h"
#include "../mt76_csi.h"

/* --- csi_enable: Start/Stop CSI capture --- */

static int
mt7615_csi_enable_set(void *data, u64 val)
{
	struct mt7615_dev *dev = data;
	struct mt76_csi_buf *csi = dev->mt76.csi_buf;
	bool enable = !!val;
	unsigned long flags;
	int ret;

	if (!csi)
		return -ENOMEM;

	if (enable == csi->capture_active)
		return 0;

	csi->capture_active = enable;

	if (enable) {
		spin_lock_irqsave(&csi->lock, flags);
		/* Reset counters on enable */
		csi->head = 0;
		csi->tail = 0;
		csi->overflow_count = 0;
		csi->total_captured = 0;
		csi->total_dropped = 0;
		spin_unlock_irqrestore(&csi->lock, flags);

		mt76_clear(dev, MT_DMA_DCR0, MT_DMA_DCR0_RX_VEC_DROP);
		ret = mt7615_mcu_set_csi(dev, true);
		if (ret)
			dev_warn(dev->mt76.dev, "mt7615_mcu_set_csi(true) returned %d\n", ret);

		dev_info(dev->mt76.dev, "CSI capture ENABLED (mode=%d)\n",
			 csi->mode);
	} else {
		mt76_set(dev, MT_DMA_DCR0, MT_DMA_DCR0_RX_VEC_DROP);
		ret = mt7615_mcu_set_csi(dev, false);
		if (ret)
			dev_warn(dev->mt76.dev, "mt7615_mcu_set_csi(false) returned %d\n", ret);

		dev_info(dev->mt76.dev,
			 "CSI capture DISABLED (captured=%u, dropped=%u)\n",
			 csi->total_captured, csi->total_dropped);
		wake_up_interruptible(&csi->wait);
	}

	return 0;
}

static int
mt7615_csi_enable_get(void *data, u64 *val)
{
	struct mt7615_dev *dev = data;

	*val = dev->mt76.csi_buf ? dev->mt76.csi_buf->capture_active : 0;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_csi_enable, mt7615_csi_enable_get,
			 mt7615_csi_enable_set, "%llu\n");

/* --- csi_mode: Set capture mode --- */

static int
mt7615_csi_mode_set(void *data, u64 val)
{
	struct mt7615_dev *dev = data;
	struct mt76_csi_buf *csi = dev->mt76.csi_buf;

	if (!csi)
		return -ENOMEM;
	if (val > MT76_CSI_MODE_METRICS)
		return -EINVAL;

	csi->mode = val;
	dev_info(dev->mt76.dev, "CSI mode set to %llu\n", val);
	return 0;
}

static int
mt7615_csi_mode_get(void *data, u64 *val)
{
	struct mt7615_dev *dev = data;

	*val = dev->mt76.csi_buf ? dev->mt76.csi_buf->mode : 0;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_csi_mode, mt7615_csi_mode_get,
			 mt7615_csi_mode_set, "%llu\n");

/* --- csi_stats: Show buffer statistics --- */

static int
mt7615_csi_stats_show(struct seq_file *s, void *data)
{
	struct mt7615_dev *dev = dev_get_drvdata(s->private);
	struct mt76_csi_buf *csi = dev->mt76.csi_buf;

	if (!csi) {
		seq_puts(s, "CSI buffer not allocated\n");
		return 0;
	}

	seq_printf(s, "capture_active: %d\n", csi->capture_active);
	seq_printf(s, "mode:           %d\n", csi->mode);
	seq_printf(s, "total_captured: %u\n", csi->total_captured);
	seq_printf(s, "total_dropped:  %u\n", csi->total_dropped);
	seq_printf(s, "overflow_count: %u\n", csi->overflow_count);
	seq_printf(s, "buf_head:       %u\n", csi->head);
	seq_printf(s, "buf_tail:       %u\n", csi->tail);
	seq_printf(s, "buf_used:       %u / %u\n",
		   csi->head - csi->tail, csi->size);
	seq_printf(s, "filter_enabled: %d\n", csi->filter_enabled);
	if (csi->filter_enabled)
		seq_printf(s, "filter_mac:     %pM\n", csi->filter_mac);

	return 0;
}

/* --- csi_filter_mac: Set MAC address filter --- */

static ssize_t
mt7615_csi_filter_read(struct file *file, char __user *user_buf,
		       size_t count, loff_t *ppos)
{
	struct mt7615_dev *dev = file->private_data;
	struct mt76_csi_buf *csi = dev->mt76.csi_buf;
	char buf[64];
	int len;

	if (!csi)
		return -ENOMEM;

	if (csi->filter_enabled)
		len = snprintf(buf, sizeof(buf), "%pM\n", csi->filter_mac);
	else
		len = snprintf(buf, sizeof(buf), "off\n");

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t
mt7615_csi_filter_write(struct file *file, const char __user *user_buf,
			size_t count, loff_t *ppos)
{
	struct mt7615_dev *dev = file->private_data;
	struct mt76_csi_buf *csi = dev->mt76.csi_buf;
	char buf[32];
	u8 mac[6];

	if (!csi)
		return -ENOMEM;
	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count] = '\0';

	/* "off" or "none" disables filter */
	if (strncmp(buf, "off", 3) == 0 || strncmp(buf, "none", 4) == 0) {
		csi->filter_enabled = false;
		dev_info(dev->mt76.dev, "CSI MAC filter disabled\n");
		return count;
	}

	if (sscanf(buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		   &mac[0], &mac[1], &mac[2],
		   &mac[3], &mac[4], &mac[5]) != 6)
		return -EINVAL;

	memcpy(csi->filter_mac, mac, 6);
	csi->filter_enabled = true;
	dev_info(dev->mt76.dev, "CSI MAC filter set to %pM\n", mac);

	return count;
}

static const struct file_operations fops_csi_filter = {
	.read = mt7615_csi_filter_read,
	.write = mt7615_csi_filter_write,
	.open = simple_open,
	.llseek = default_llseek,
};

/* --- csi_data: Read raw CSI data (binary) --- */

static ssize_t
mt7615_csi_data_read(struct file *file, char __user *user_buf,
		     size_t count, loff_t *ppos)
{
	struct mt7615_dev *dev = file->private_data;
	struct mt76_csi_buf *csi = dev->mt76.csi_buf;
	struct mt76_csi_data *entry;
	int ret;

	if (!csi)
		return -ENOMEM;

	/* Wait for data if buffer is empty */
	if (csi->head == csi->tail) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(csi->wait,
					       csi->head != csi->tail ||
					       !csi->capture_active);
		if (ret)
			return ret;
		if (!csi->capture_active && csi->head == csi->tail)
			return 0;
	}

	if (count < sizeof(*entry))
		return -EINVAL;

	entry = kmalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	ret = mt76_csi_buf_read(csi, entry, 1);
	if (ret <= 0) {
		kfree(entry);
		return -EAGAIN;
	}

	if (copy_to_user(user_buf, entry, sizeof(*entry))) {
		kfree(entry);
		return -EFAULT;
	}

	kfree(entry);
	return sizeof(*entry);
}

static __poll_t
mt7615_csi_data_poll(struct file *file, struct poll_table_struct *wait)
{
	struct mt7615_dev *dev = file->private_data;
	struct mt76_csi_buf *csi = dev->mt76.csi_buf;

	if (!csi)
		return EPOLLERR;

	poll_wait(file, &csi->wait, wait);

	if (csi->head != csi->tail)
		return EPOLLIN | EPOLLRDNORM;

	return 0;
}

static const struct file_operations fops_csi_data = {
	.read = mt7615_csi_data_read,
	.poll = mt7615_csi_data_poll,
	.open = simple_open,
	.llseek = noop_llseek,
};

/* --- Registration function called from mt7615 debugfs init --- */

void mt7615_csi_debugfs_register(struct mt7615_dev *dev, struct dentry *dir)
{
	debugfs_create_file("csi_enable", 0600, dir, dev, &fops_csi_enable);
	debugfs_create_file("csi_mode", 0600, dir, dev, &fops_csi_mode);
	debugfs_create_devm_seqfile(dev->mt76.dev, "csi_stats", dir, mt7615_csi_stats_show);
	debugfs_create_file("csi_filter_mac", 0600, dir, dev,
			    &fops_csi_filter);
	debugfs_create_file("csi_data", 0400, dir, dev, &fops_csi_data);
}
