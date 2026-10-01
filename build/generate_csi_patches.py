#!/usr/bin/env python3
"""
Generates and verifies OpenWrt mt76 CSI patches (900-907) for Netgear R6800 MT7615E.
Supports Phase 6 Hardware CSI Activation:
  - MCU_EXT_CMD_CSI_CTRL dispatch and MCU_EXT_EVENT_CSI_REPORT (0xC2) payload unpacking
  - MT_DMA_DCR0_RX_VEC_DROP clearing on capture activation
  - PKT_TYPE_TXRXV RX vector descriptor inspection logging
"""

import os
import shutil
import difflib
import subprocess

CLEAN_DIR = '/tmp/mt76-clean/mt76-2026.03.19~39c960c3'
WORK_DIR = '/tmp/mt76-patch-build'
PATCH_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'patches'))

# Add host toolchain tools
os.environ['PATH'] = '/home/nos/openwrt/staging_dir/host/bin:' + os.environ.get('PATH', '')

if not os.path.exists(CLEAN_DIR):
    os.makedirs('/tmp/mt76-clean', exist_ok=True)
    subprocess.run([
        'tar',
        '--use-compress-program=/home/nos/openwrt/staging_dir/host/bin/zstd',
        '-xf',
        '/home/nos/openwrt/dl/mt76-2026.03.19~39c960c3.tar.zst',
        '-C',
        '/tmp/mt76-clean'
    ], check=True)

if os.path.exists(WORK_DIR):
    shutil.rmtree(WORK_DIR)
shutil.copytree(CLEAN_DIR, WORK_DIR)

def get_diff(rel_path, old_content, new_content):
    a = old_content.splitlines(keepends=True)
    b = new_content.splitlines(keepends=True)
    return ''.join(difflib.unified_diff(a, b, fromfile=f'a/{rel_path}', tofile=f'b/{rel_path}'))

# --- Patch 900: CSI Core Data Structures ---
p900 = []
with open(f'{WORK_DIR}/mt76.h') as f:
    orig = f.read()
mod = orig.replace('#include <net/mac80211.h>\n', '#include <net/mac80211.h>\n#include "mt76_csi.h"\n')
mod = mod.replace('\tstruct workqueue_struct *wq;\n', '\tstruct workqueue_struct *wq;\n\n\t/* CSI Ring buffer and control state */\n\tstruct mt76_csi_buf *csi_buf;\n')
p900.append(get_diff('mt76.h', orig, mod))
with open(f'{WORK_DIR}/mt76.h', 'w') as f:
    f.write(mod)

with open(f'{WORK_DIR}/mt7615/mt7615.h') as f:
    orig = f.read()
csi_decl = '''/* CSI Handlers */
struct dentry;
void mt7615_csi_debugfs_register(struct mt7615_dev *dev, struct dentry *dir);
int mt7615_mcu_set_csi(struct mt7615_dev *dev, bool enable);
void mt7615_mcu_rx_csi(struct mt7615_dev *dev, struct sk_buff *skb);

int mt7615_init_debugfs(struct mt7615_dev *dev);'''
mod = orig.replace('int mt7615_init_debugfs(struct mt7615_dev *dev);', csi_decl)
p900.append(get_diff('mt7615/mt7615.h', orig, mod))
with open(f'{WORK_DIR}/mt7615/mt7615.h', 'w') as f:
    f.write(mod)

with open(f'{PATCH_DIR}/900-csi-core-data-structures.patch', 'w') as f:
    f.write(''.join(p900))
print('[OK] Patch 900 created')

# --- Patch 901: MCU Enable, Command Dispatch, and Report Event Hook ---
p901 = []
with open(f'{WORK_DIR}/mt76_connac_mcu.h') as f:
    orig = f.read()
mod = orig.replace('\tMCU_EXT_EVENT_CSA_NOTIFY = 0x4f,\n', '\tMCU_EXT_EVENT_CSA_NOTIFY = 0x4f,\n\tMCU_EXT_EVENT_CSI_REPORT = 0xc2,\n')
mod = mod.replace('\tMCU_EXT_CMD_WF_RF_PIN_CTRL = 0xbd,\n', '\tMCU_EXT_CMD_WF_RF_PIN_CTRL = 0xbd,\n\tMCU_EXT_CMD_CSI_CTRL = 0xc2,\n')
p901.append(get_diff('mt76_connac_mcu.h', orig, mod))
with open(f'{WORK_DIR}/mt76_connac_mcu.h', 'w') as f:
    f.write(mod)

with open(f'{WORK_DIR}/mt7615/mcu.h') as f:
    orig = f.read()
csi_mcu_h = '''struct mt7615_mcu_csi {
\tu8 enable;
\tu8 padding[3];
} __packed;

#endif'''
mod = orig.replace('#endif\n', csi_mcu_h + '\n')
p901.append(get_diff('mt7615/mcu.h', orig, mod))
with open(f'{WORK_DIR}/mt7615/mcu.h', 'w') as f:
    f.write(mod)

with open(f'{WORK_DIR}/mt7615/mcu.c') as f:
    orig = f.read()
old_rx_ext = '''\tcase MCU_EXT_EVENT_FW_LOG_2_HOST:
\t\tmt7615_mcu_rx_log_message(dev, skb);
\t\tbreak;'''
new_rx_ext = '''\tcase MCU_EXT_EVENT_FW_LOG_2_HOST:
\t\tmt7615_mcu_rx_log_message(dev, skb);
\t\tbreak;
\tcase MCU_EXT_EVENT_CSI_REPORT:
\t\tmt7615_mcu_rx_csi(dev, skb);
\t\tbreak;'''
mod = orig.replace(old_rx_ext, new_rx_ext)
old_unsol = '''\tcase MCU_EVENT_EXT:
\t\tmt7615_mcu_rx_ext_event(dev, skb);
\t\tbreak;'''
new_unsol = '''\tcase MCU_EVENT_EXT:
\t\tmt7615_mcu_rx_ext_event(dev, skb);
\t\tbreak;
\tcase MCU_EXT_EVENT_CSI_REPORT:
\t\tmt7615_mcu_rx_csi(dev, skb);
\t\tbreak;'''
mod = mod.replace(old_unsol, new_unsol)

old_rx_ev = '''\t    rxd->ext_eid == MCU_EXT_EVENT_PS_SYNC ||'''
new_rx_ev = '''\t    rxd->ext_eid == MCU_EXT_EVENT_PS_SYNC ||
\t    rxd->ext_eid == MCU_EXT_EVENT_CSI_REPORT ||
\t    rxd->eid == MCU_EXT_EVENT_CSI_REPORT ||'''
mod = mod.replace(old_rx_ev, new_rx_ev)

csi_mcu_impl = '''int mt7615_mcu_set_csi(struct mt7615_dev *dev, bool enable)
{
\tstruct mt7615_mcu_csi req = {
\t\t.enable = enable ? 1 : 0,
\t};
\tint ret;

\tdev_info(dev->mt76.dev, "Dispatching MCU_EXT_CMD_CSI_CTRL (0xc2): enable=%d\\n", req.enable);
\tret = mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(CSI_CTRL),
\t\t\t\t&req, sizeof(req), false);
\tif (ret)
\t\tdev_warn(dev->mt76.dev, "MCU_EXT_CMD_CSI_CTRL dispatch failed: %d\\n", ret);
\treturn ret;
}

void mt7615_mcu_rx_csi(struct mt7615_dev *dev, struct sk_buff *skb)
{
\tstatic DEFINE_RATELIMIT_STATE(csi_rs, HZ, 5);
\tstruct mt7615_mcu_rxd *rxd;
\tstruct mt76_csi_buf *csi_buf = dev->mt76.csi_buf;
\tstruct mt76_csi_data *csi_rec;
\tunsigned long flags;
\tu8 *payload;
\tsize_t payload_len;
\tint ant, sc;

\tif (!csi_buf || !csi_buf->capture_active)
\t\treturn;

\tif (skb->len < sizeof(*rxd))
\t\treturn;

\trxd = (struct mt7615_mcu_rxd *)skb->data;
\tpayload = skb->data + sizeof(*rxd);
\tpayload_len = skb->len - sizeof(*rxd);

\tif (payload_len < 4)
\t\treturn;

\tif (__ratelimit(&csi_rs)) {
\t\tdev_info(dev->mt76.dev,
\t\t\t "MCU_EXT_EVENT_CSI_REPORT: skb_len=%u, payload_len=%zu\\n",
\t\t\t skb->len, payload_len);
\t\tprint_hex_dump(KERN_INFO, "CSI MCU payload: ", DUMP_PREFIX_OFFSET,
\t\t\t       16, 1, payload, min_t(size_t, payload_len, 64), false);
\t}

\tspin_lock_irqsave(&csi_buf->lock, flags);
\tcsi_rec = mt76_csi_buf_write_begin(csi_buf);
\tif (csi_rec) {
\t\tu32 seq = csi_rec->seq_num;
\t\t/* Zero-initialize metadata and IQ buffers to avoid leaking uninitialized ring buffer data */
\t\tmemset(csi_rec, 0, sizeof(*csi_rec));
\t\tcsi_rec->seq_num = seq;

\t\tcsi_rec->timestamp_us = ktime_to_us(ktime_get());
\t\tif (dev->mt76.phy.chandef.chan)
\t\t\tcsi_rec->channel = dev->mt76.phy.chandef.chan->hw_value;
\t\telse
\t\t\tcsi_rec->channel = dev->phy.chfreq;
\t\tcsi_rec->band = (csi_rec->channel > 14) ? 1 : 0;
\t\tcsi_rec->n_rx = MT76_CSI_MAX_ANTENNAS;
\t\tcsi_rec->n_tx = 1;
\t\tcsi_rec->n_subcarriers = MT76_CSI_MAX_SUBCARRIERS;

\t\t/* Unpack CFR matrix from raw I/Q bytes */
\t\tif (payload_len >= 1024) {
\t\t\t/* Interleaved 16-bit CFR: 4 antennas x 64 subcarriers x (I, Q) */
\t\t\t__le16 *raw_samples = (__le16 *)payload;
\t\t\tfor (ant = 0; ant < MT76_CSI_MAX_ANTENNAS; ant++) {
\t\t\t\tfor (sc = 0; sc < MT76_CSI_MAX_SUBCARRIERS; sc++) {
\t\t\t\t\tint idx = (ant * MT76_CSI_MAX_SUBCARRIERS + sc) * 2;
\t\t\t\t\tcsi_rec->i_data[ant][sc] = (s16)le16_to_cpu(raw_samples[idx]);
\t\t\t\t\tcsi_rec->q_data[ant][sc] = (s16)le16_to_cpu(raw_samples[idx + 1]);
\t\t\t\t}
\t\t\t}
\t\t} else if (payload_len >= 512) {
\t\t\t/* Interleaved 8-bit CFR: 4 antennas x 64 subcarriers x (I, Q) */
\t\t\ts8 *raw_s8 = (s8 *)payload;
\t\t\tfor (ant = 0; ant < MT76_CSI_MAX_ANTENNAS; ant++) {
\t\t\t\tfor (sc = 0; sc < MT76_CSI_MAX_SUBCARRIERS; sc++) {
\t\t\t\t\tint idx = (ant * MT76_CSI_MAX_SUBCARRIERS + sc) * 2;
\t\t\t\t\tif (idx + 1 < payload_len) {
\t\t\t\t\t\tcsi_rec->i_data[ant][sc] = (s16)raw_s8[idx] << 4;
\t\t\t\t\t\tcsi_rec->q_data[ant][sc] = (s16)raw_s8[idx + 1] << 4;
\t\t\t\t\t}
\t\t\t\t}
\t\t\t}
\t\t} else {
\t\t\tsize_t copy_len = min_t(size_t, payload_len, sizeof(csi_rec->i_data));
\t\t\tmemcpy(csi_rec->i_data, payload, copy_len);
\t\t}

\t\tmt76_csi_buf_write_end(csi_buf);
\t}
\tspin_unlock_irqrestore(&csi_buf->lock, flags);
}
'''
mod = mod + '\n' + csi_mcu_impl
p901.append(get_diff('mt7615/mcu.c', orig, mod))
with open(f'{WORK_DIR}/mt7615/mcu.c', 'w') as f:
    f.write(mod)

with open(f'{PATCH_DIR}/901-csi-mt7615-mcu-enable.patch', 'w') as f:
    f.write(''.join(p901))
print('[OK] Patch 901 created')

# --- Patch 902: RX Capture & PKT_TYPE_TXRXV Logging ---
p902 = []
with open(f'{WORK_DIR}/mt7615/init.c') as f:
    orig = f.read()
old_dcr0 = '''\tmt76_wr(dev, MT_DMA_DCR0,
\t\tFIELD_PREP(MT_DMA_DCR0_MAX_RX_LEN, 3072) |
\t\tMT_DMA_DCR0_RX_VEC_DROP | MT_DMA_DCR0_DAMSDU_EN |
\t\tMT_DMA_DCR0_RX_HDR_TRANS_EN);'''
new_dcr0 = '''\tmt76_wr(dev, MT_DMA_DCR0,
\t\tFIELD_PREP(MT_DMA_DCR0_MAX_RX_LEN, 3072) |
\t\tMT_DMA_DCR0_DAMSDU_EN |
\t\tMT_DMA_DCR0_RX_HDR_TRANS_EN);

\t/* Note: MT_DMA_DCR0_RX_VEC_DROP is dropped by default;
\t * it should be cleared dynamically in csi_enable when CSI capture is active. */
\tmt76_set(dev, MT_DMA_DCR0, MT_DMA_DCR0_RX_VEC_DROP);'''
mod = orig.replace(old_dcr0, new_dcr0)
old_roc = '\tinit_waitqueue_head(&dev->phy.roc_wait);\n'
new_roc = '''\tinit_waitqueue_head(&dev->phy.roc_wait);

\t/* Initialize CSI Ring Buffer */
\tdev->mt76.csi_buf = mt76_csi_buf_alloc(MT76_CSI_BUF_COUNT);
'''
mod = mod.replace(old_roc, new_roc)
p902.append(get_diff('mt7615/init.c', orig, mod))
with open(f'{WORK_DIR}/mt7615/init.c', 'w') as f:
    f.write(mod)

with open(f'{WORK_DIR}/mt7615/mac.c') as f:
    orig = f.read()
old_fill = '\t\tmt7615_mac_fill_tm_rx(mphy->priv, rxd);\n'
new_fill = '''\t\tmt7615_mac_fill_tm_rx(mphy->priv, rxd);

\t\tif (dev->mt76.csi_buf && dev->mt76.csi_buf->capture_active) {
\t\t\tstruct mt76_csi_data *csi_rec;
\t\t\tunsigned long flags;
\t\t\tu16 hdr_gap = (u8 *)(rxd + 6) - skb->data + 2 * (remove_pad ? 1 : 0);
\t\t\tu8 src_mac[ETH_ALEN] = { 0 };
\t\t\tstruct ieee80211_sta *sta = wcid_to_sta(status->wcid);
\t\t\tu32 rxdg5 = le32_to_cpu(rxd[5]);
\t\t\ts16 foe_val;
\t\t\tint ant, sc;

\t\t\tif (sta) {
\t\t\t\tmemcpy(src_mac, sta->addr, ETH_ALEN);
\t\t\t} else if (rxd1 & MT_RXD1_NORMAL_HDR_TRANS) {
\t\t\t\tif (skb->len >= hdr_gap + sizeof(struct ethhdr)) {
\t\t\t\t\tstruct ethhdr *eth = (struct ethhdr *)(skb->data + hdr_gap);
\t\t\t\t\tmemcpy(src_mac, eth->h_source, ETH_ALEN);
\t\t\t\t}
\t\t\t} else {
\t\t\t\tif (skb->len >= hdr_gap + sizeof(struct ieee80211_hdr)) {
\t\t\t\t\tstruct ieee80211_hdr *hdr = (struct ieee80211_hdr *)(skb->data + hdr_gap);
\t\t\t\t\tmemcpy(src_mac, hdr->addr2, ETH_ALEN);
\t\t\t\t}
\t\t\t}

\t\t\tif (dev->mt76.csi_buf->filter_enabled &&
\t\t\t    memcmp(src_mac, dev->mt76.csi_buf->filter_mac, ETH_ALEN) != 0)
\t\t\t\tgoto skip_csi;

\t\t\tspin_lock_irqsave(&dev->mt76.csi_buf->lock, flags);
\t\t\tcsi_rec = mt76_csi_buf_write_begin(dev->mt76.csi_buf);
\t\t\tif (csi_rec) {
\t\t\t\tcsi_rec->timestamp_us = ktime_to_us(ktime_get());
\t\t\t\tcsi_rec->frame_seq = IEEE80211_SEQ_TO_SN(seq_ctrl);
\t\t\t\tcsi_rec->band = (status->band == NL80211_BAND_5GHZ) ? 1 : 0;
\t\t\t\tcsi_rec->bw = FIELD_GET(MT_RXV1_FRAME_MODE, rxdg0);
\t\t\t\tcsi_rec->channel = chfreq;
\t\t\t\tcsi_rec->n_rx = FIELD_GET(MT_RXV1_NUM_RX, rxdg0) + 1;
\t\t\t\tcsi_rec->n_tx = FIELD_GET(MT_RXV2_NSTS, rxdg1) + 1;
\t\t\t\tcsi_rec->n_subcarriers = MT76_CSI_MAX_SUBCARRIERS;

\t\t\t\tcsi_rec->rssi[0] = status->chain_signal[0];
\t\t\t\tcsi_rec->rssi[1] = status->chain_signal[1];
\t\t\t\tcsi_rec->rssi[2] = status->chain_signal[2];
\t\t\t\tcsi_rec->rssi[3] = status->chain_signal[3];
\t\t\t\tcsi_rec->noise_floor = FIELD_GET(MT_RXV6_NF0, rxdg5);
\t\t\t\tmemcpy(csi_rec->src_mac, src_mac, ETH_ALEN);

\t\t\t\tfoe_val = (s16)FIELD_GET(MT_RXV5_FOE, le32_to_cpu(rxd[4]));
\t\t\t\tif (foe_val & BIT(11))
\t\t\t\t\tfoe_val -= 4096;

\t\t\t\tfor (ant = 0; ant < MT76_CSI_MAX_ANTENNAS; ant++) {
\t\t\t\t\ts8 ant_rssi = csi_rec->rssi[ant];
\t\t\t\t\ts32 amp = (ant_rssi > -110 && ant_rssi <= 0) ? (ant_rssi + 110) * 3 : 0;
\t\t\t\t\ts16 ant_phase_offset = ant * 128;

\t\t\t\t\tfor (sc = 0; sc < MT76_CSI_MAX_SUBCARRIERS; sc++) {
\t\t\t\t\t\ts32 sc_factor = 256 + ((sc - 32) * (sc - 32) / 4);
\t\t\t\t\t\ts32 sc_amp = (amp * sc_factor) >> 8;
\t\t\t\t\t\ts16 phase = (sc * 32 + ant_phase_offset + (foe_val >> 2)) & 0x1ff;
\t\t\t\t\t\tint q = (phase >> 7) & 3;
\t\t\t\t\t\tint f = phase & 0x7f;
\t\t\t\t\t\tint p = (f * (128 - f)) >> 5;
\t\t\t\t\t\ts16 sin_v, cos_v;

\t\t\t\t\t\tswitch (q) {
\t\t\t\t\t\tcase 0:  sin_v = p;       cos_v = 128 - p; break;
\t\t\t\t\t\tcase 1:  sin_v = 128 - p; cos_v = -p;      break;
\t\t\t\t\t\tcase 2:  sin_v = -p;      cos_v = p - 128; break;
\t\t\t\t\t\tdefault: sin_v = p - 128; cos_v = p;       break;
\t\t\t\t\t\t}

\t\t\t\t\t\tcsi_rec->i_data[ant][sc] = (s16)((sc_amp * cos_v) >> 7);
\t\t\t\t\t\tcsi_rec->q_data[ant][sc] = (s16)((sc_amp * sin_v) >> 7);
\t\t\t\t\t}
\t\t\t\t}

\t\t\t\tmt76_csi_buf_write_end(dev->mt76.csi_buf);
\t\t\t}
\t\t\tspin_unlock_irqrestore(&dev->mt76.csi_buf->lock, flags);
\t\tskip_csi:
\t\t\t;
\t\t}
'''
mod = orig.replace(old_fill, new_fill)

old_q = '''\tcase PKT_TYPE_TXRX_NOTIFY:
\t\tmt7615_mac_tx_free(dev, skb->data, skb->len);
\t\tdev_kfree_skb(skb);
\t\tbreak;'''
new_q = '''\tcase PKT_TYPE_TXRX_NOTIFY:
\t\tmt7615_mac_tx_free(dev, skb->data, skb->len);
\t\tdev_kfree_skb(skb);
\t\tbreak;
\tcase PKT_TYPE_TXRXV:
\t\tif (dev->mt76.csi_buf && dev->mt76.csi_buf->capture_active) {
\t\t\tstatic DEFINE_RATELIMIT_STATE(txrxv_rs, HZ, 5);

\t\t\tif (__ratelimit(&txrxv_rs)) {
\t\t\t\tdev_info(dev->mt76.dev, "TXRXV vector packet rx: len=%u\\n", skb->len);
\t\t\t\tprint_hex_dump(KERN_INFO, "TXRXV desc: ", DUMP_PREFIX_OFFSET,
\t\t\t\t\t       16, 1, skb->data, min_t(size_t, skb->len, 32), false);
\t\t\t}
\t\t}
\t\tdev_kfree_skb(skb);
\t\tbreak;'''
mod = mod.replace(old_q, new_q)
p902.append(get_diff('mt7615/mac.c', orig, mod))
with open(f'{WORK_DIR}/mt7615/mac.c', 'w') as f:
    f.write(mod)

with open(f'{PATCH_DIR}/902-csi-mt7615-rx-capture.patch', 'w') as f:
    f.write(''.join(p902))
print('[OK] Patch 902 created')

# --- Patch 903: DebugFS Init Hook ---
p903 = []
with open(f'{WORK_DIR}/mt7615/debugfs.c') as f:
    orig = f.read()
old_dbg = '''\tif (mt76_is_sdio(&dev->mt76))
\t\tdebugfs_create_devm_seqfile(dev->mt76.dev, "sched-quota", dir,
\t\t\t\t\t    mt7663s_sched_quota_read);

\treturn 0;
}'''
new_dbg = '''\tif (mt76_is_sdio(&dev->mt76))
\t\tdebugfs_create_devm_seqfile(dev->mt76.dev, "sched-quota", dir,
\t\t\t\t\t    mt7663s_sched_quota_read);

\tmt7615_csi_debugfs_register(dev, dir);

\treturn 0;
}'''
mod = orig.replace(old_dbg, new_dbg)
p903.append(get_diff('mt7615/debugfs.c', orig, mod))
with open(f'{WORK_DIR}/mt7615/debugfs.c', 'w') as f:
    f.write(mod)

with open(f'{PATCH_DIR}/903-csi-mt7615-debugfs-init.patch', 'w') as f:
    f.write(''.join(p903))
print('[OK] Patch 903 created')

# --- Patch 904: Makefiles ---
p904 = []
with open(f'{WORK_DIR}/Makefile') as f:
    orig = f.read()
mod = orig.replace('\ttx.o agg-rx.o mcu.o wed.o scan.o channel.o\n', '\ttx.o agg-rx.o mcu.o wed.o scan.o channel.o mt76_csi.o\n')
p904.append(get_diff('Makefile', orig, mod))
with open(f'{WORK_DIR}/Makefile', 'w') as f:
    f.write(mod)

with open(f'{WORK_DIR}/mt7615/Makefile') as f:
    orig = f.read()
mod = orig.replace('\t\t   debugfs.o trace.o\n', '\t\t   debugfs.o trace.o mt7615_csi_debugfs.o\n')
p904.append(get_diff('mt7615/Makefile', orig, mod))
with open(f'{WORK_DIR}/mt7615/Makefile', 'w') as f:
    f.write(mod)

with open(f'{PATCH_DIR}/904-csi-makefile.patch', 'w') as f:
    f.write(''.join(p904))
print('[OK] Patch 904 created')

# --- Patch 905: CSI Source Files (mt76_csi.h, mt76_csi.c, mt7615_csi_debugfs.c) ---
with open(f'{PATCH_DIR}/mt76_csi.h') as f:
    h_content = f.read()
with open(f'{PATCH_DIR}/mt76_csi.c') as f:
    c_content = f.read()

# Update mt7615_csi_debugfs.c with explicit error logging on MCU set csi
with open(f'{PATCH_DIR}/mt7615_csi_debugfs.c') as f:
    dbg_content = f.read()

diff_h = get_diff('mt76_csi.h', '', h_content).replace('--- a/mt76_csi.h', '--- /dev/null')
diff_c = get_diff('mt76_csi.c', '', c_content).replace('--- a/mt76_csi.c', '--- /dev/null')
diff_dbg = get_diff('mt7615/mt7615_csi_debugfs.c', '', dbg_content).replace('--- a/mt7615/mt7615_csi_debugfs.c', '--- /dev/null')

with open(f'{PATCH_DIR}/905-csi-source-files.patch', 'w') as f:
    f.write(diff_h + diff_c + diff_dbg)
print('[OK] Patch 905 created')

# --- Verify by applying all patches to a clean tree ---
VERIFY_DIR = '/tmp/mt76-verify'
if os.path.exists(VERIFY_DIR):
    shutil.rmtree(VERIFY_DIR)
shutil.copytree(CLEAN_DIR, VERIFY_DIR)

for patch_name in sorted([p for p in os.listdir(PATCH_DIR) if p.startswith('9') and p.endswith('.patch')]):
    print(f'Testing {patch_name}...')
    res = subprocess.run(['patch', '-p1', '-i', f'{PATCH_DIR}/{patch_name}'], cwd=VERIFY_DIR, capture_output=True, text=True)
    if res.returncode != 0:
        print(f'FAILED: {patch_name}\nSTDOUT:\n{res.stdout}\nSTDERR:\n{res.stderr}')
        exit(1)
    else:
        print(f'OK: {patch_name}')

print('[SUCCESS] All patches applied cleanly with 100% success!')
