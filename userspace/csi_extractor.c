/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Netgear R6800 MT7615 CSI Userspace Extraction Daemon
 *
 * Polls /sys/kernel/debug/ieee80211/<phy>/mt76/csi_data,
 * reads binary struct mt76_csi_data frames (1058 bytes),
 * and streams them via non-blocking UDP to a remote laptop.
 *
 * Copyright (C) 2026 R6800 CSI Project
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <poll.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MT76_CSI_MAX_SUBCARRIERS 64
#define MT76_CSI_MAX_ANTENNAS    4
#define DEFAULT_PORT             5500
#define DEFAULT_PHY              "phy5"
#define DEFAULT_SNDBUF_SIZE      (512 * 1024)

#pragma pack(push, 1)
/*
 * struct mt76_csi_data - 1058 bytes matching kernel mt76_csi.h
 */
struct mt76_csi_data {
	/* Timing */
	uint64_t timestamp_us;              /* Kernel ktime_get_ns() / 1000 */
	uint32_t seq_num;                   /* Monotonic sequence number */
	uint16_t frame_seq;                 /* 802.11 MAC sequence number */

	/* Wireless Metadata */
	uint8_t  band;                      /* 0=2.4GHz, 1=5GHz */
	uint8_t  bw;                        /* 0=20MHz, 1=40MHz, 2=80MHz */
	uint8_t  channel;                   /* Channel number */
	uint8_t  n_rx;                      /* Number of Rx antennas used */
	uint8_t  n_tx;                      /* Number of detected Tx antennas */
	uint8_t  n_subcarriers;             /* Number of subcarriers reported */
	int8_t   rssi[MT76_CSI_MAX_ANTENNAS];/* Per-antenna RSSI (dBm) */
	uint8_t  noise_floor;               /* Noise floor (dBm) */
	uint8_t  _pad0;

	/* Source identification */
	uint8_t  src_mac[6];                /* Transmitter MAC address */
	uint8_t  _pad1[2];

	/* I/Q Data: [antenna][subcarrier] */
	int16_t  i_data[MT76_CSI_MAX_ANTENNAS][MT76_CSI_MAX_SUBCARRIERS];
	int16_t  q_data[MT76_CSI_MAX_ANTENNAS][MT76_CSI_MAX_SUBCARRIERS];
};
#pragma pack(pop)

_Static_assert(sizeof(struct mt76_csi_data) == 1058,
	       "Error: struct mt76_csi_data must be exactly 1058 bytes");

/* Global running flag for signals */
static volatile sig_atomic_t g_running = 1;

static void handle_signal(int sig)
{
	(void)sig;
	g_running = 0;
}

struct extractor_config {
	char phy_name[32];
	char data_path[256];
	char enable_path[256];
	char target_ip[64];
	uint16_t target_port;
	int sndbuf_size;
	bool auto_enable;
	bool verbose;
	int stats_interval_s;
};

struct extractor_stats {
	uint64_t total_read;
	uint64_t total_sent;
	uint64_t bytes_sent;
	uint64_t send_dropped;
	uint64_t send_errors;
	uint64_t incomplete_reads;
	uint32_t last_seq;
	uint64_t seq_gaps;
	bool first_pkt;
};

static void print_usage(const char *prog)
{
	fprintf(stderr,
		"Netgear R6800 MT7615 CSI Extraction Daemon\n\n"
		"Usage: %s -d <target_ip> [options]\n\n"
		"Required:\n"
		"  -d <ip>       Destination IPv4 address (e.g. 192.168.10.100)\n\n"
		"Options:\n"
		"  -i <phy>      Wireless PHY interface name (default: %s)\n"
		"  -f <path>     Explicit path to csi_data DebugFS node\n"
		"  -p <port>     Destination UDP port (default: %u)\n"
		"  -e            Auto-enable CSI on startup (echo 1 > csi_enable)\n"
		"                and auto-disable on exit (echo 0 > csi_enable)\n"
		"  -b <bytes>    Socket SO_SNDBUF size (default: %d bytes)\n"
		"  -s <sec>      Stats report interval in seconds (default: 1, 0=off)\n"
		"  -v            Verbose mode (print details per packet)\n"
		"  -h            Show this help text\n\n"
		"Example:\n"
		"  %s -i phy5 -d 192.168.10.100 -p 5500 -e\n",
		prog, DEFAULT_PHY, DEFAULT_PORT, DEFAULT_SNDBUF_SIZE, prog);
}

static int set_csi_enable(const char *enable_path, int val)
{
	int fd = open(enable_path, O_WRONLY);
	if (fd < 0) {
		fprintf(stderr, "Warning: Failed to open %s: %s\n",
			enable_path, strerror(errno));
		return -1;
	}

	char buf[8];
	int len = snprintf(buf, sizeof(buf), "%d\n", val);
	if (write(fd, buf, len) < 0) {
		fprintf(stderr, "Warning: Failed to write %d to %s: %s\n",
			val, enable_path, strerror(errno));
		close(fd);
		return -1;
	}

	close(fd);
	return 0;
}

static uint64_t get_time_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000ULL + (ts.tv_nsec / 1000000ULL);
}

int main(int argc, char *argv[])
{
	struct extractor_config cfg;
	memset(&cfg, 0, sizeof(cfg));
	strncpy(cfg.phy_name, DEFAULT_PHY, sizeof(cfg.phy_name) - 1);
	cfg.target_port = DEFAULT_PORT;
	cfg.sndbuf_size = DEFAULT_SNDBUF_SIZE;
	cfg.auto_enable = false;
	cfg.verbose = false;
	cfg.stats_interval_s = 1;

	int opt;
	while ((opt = getopt(argc, argv, "d:i:f:p:eb:s:vh")) != -1) {
		switch (opt) {
		case 'd':
			strncpy(cfg.target_ip, optarg, sizeof(cfg.target_ip) - 1);
			break;
		case 'i':
			strncpy(cfg.phy_name, optarg, sizeof(cfg.phy_name) - 1);
			break;
		case 'f':
			strncpy(cfg.data_path, optarg, sizeof(cfg.data_path) - 1);
			break;
		case 'p':
			cfg.target_port = (uint16_t)atoi(optarg);
			break;
		case 'e':
			cfg.auto_enable = true;
			break;
		case 'b':
			cfg.sndbuf_size = atoi(optarg);
			break;
		case 's':
			cfg.stats_interval_s = atoi(optarg);
			break;
		case 'v':
			cfg.verbose = true;
			break;
		case 'h':
		default:
			print_usage(argv[0]);
			return (opt == 'h') ? 0 : 1;
		}
	}

	if (strlen(cfg.target_ip) == 0) {
		fprintf(stderr, "Error: Destination IP (-d <ip>) is required.\n\n");
		print_usage(argv[0]);
		return 1;
	}

	if (strlen(cfg.data_path) == 0) {
		snprintf(cfg.data_path, sizeof(cfg.data_path),
			 "/sys/kernel/debug/ieee80211/%s/mt76/csi_data",
			 cfg.phy_name);
	}

	snprintf(cfg.enable_path, sizeof(cfg.enable_path),
		 "/sys/kernel/debug/ieee80211/%s/mt76/csi_enable",
		 cfg.phy_name);

	/* Set up signal handling */
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = handle_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);

	/* Check DebugFS node existence */
	if (access(cfg.data_path, R_OK) != 0) {
		fprintf(stderr, "Error: CSI DebugFS node not accessible: %s (%s)\n",
			cfg.data_path, strerror(errno));
		fprintf(stderr, "Check if DebugFS is mounted ('mount -t debugfs none /sys/kernel/debug')\n"
				"and mt76 CSI patch is active on %s.\n", cfg.phy_name);
		return 1;
	}

	/* Auto-enable CSI capture if requested */
	if (cfg.auto_enable) {
		printf("[+] Enabling CSI capture on %s...\n", cfg.enable_path);
		if (set_csi_enable(cfg.enable_path, 1) != 0) {
			fprintf(stderr, "Warning: Could not enable CSI via %s\n",
				cfg.enable_path);
		}
	}

	/* Open CSI DebugFS stream non-blocking */
	int csi_fd = open(cfg.data_path, O_RDONLY | O_NONBLOCK);
	if (csi_fd < 0) {
		fprintf(stderr, "Error: open(%s) failed: %s\n",
			cfg.data_path, strerror(errno));
		return 1;
	}

	/* Create UDP socket */
	int sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
	if (sock_fd < 0) {
		fprintf(stderr, "Error: socket(SOCK_DGRAM) failed: %s\n",
			strerror(errno));
		close(csi_fd);
		return 1;
	}

	/* Configure socket send buffer */
	if (cfg.sndbuf_size > 0) {
		if (setsockopt(sock_fd, SOL_SOCKET, SO_SNDBUF,
			       &cfg.sndbuf_size, sizeof(cfg.sndbuf_size)) < 0) {
			fprintf(stderr, "Warning: setsockopt(SO_SNDBUF) failed: %s\n",
				strerror(errno));
		}
	}

	/* Target address */
	struct sockaddr_in target_addr;
	memset(&target_addr, 0, sizeof(target_addr));
	target_addr.sin_family = AF_INET;
	target_addr.sin_port = htons(cfg.target_port);
	if (inet_pton(AF_INET, cfg.target_ip, &target_addr.sin_addr) <= 0) {
		fprintf(stderr, "Error: Invalid target IP address '%s'\n",
			cfg.target_ip);
		close(sock_fd);
		close(csi_fd);
		return 1;
	}

	printf("============================================================\n");
	printf("  Netgear R6800 MT7615 CSI Userspace Extractor Active\n");
	printf("============================================================\n");
	printf("  Source DebugFS: %s\n", cfg.data_path);
	printf("  Destination:    %s:%u (UDP)\n", cfg.target_ip, cfg.target_port);
	printf("  Record Size:    %zu bytes\n", sizeof(struct mt76_csi_data));
	printf("  Auto-Enable:    %s\n", cfg.auto_enable ? "YES" : "NO");
	printf("  Press Ctrl+C to terminate cleanly.\n");
	printf("============================================================\n\n");

	struct extractor_stats stats;
	memset(&stats, 0, sizeof(stats));
	stats.first_pkt = true;

	struct mt76_csi_data entry;
	struct pollfd pfd;
	pfd.fd = csi_fd;
	pfd.events = POLLIN | POLLRDNORM;

	uint64_t last_report_ms = get_time_ms();
	uint64_t start_ms = last_report_ms;
	uint64_t interval_pkts = 0;
	uint64_t interval_bytes = 0;

	while (g_running) {
		int ret = poll(&pfd, 1, 500); /* 500ms timeout */
		uint64_t now_ms = get_time_ms();

		if (ret > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
			fprintf(stderr, "[-] CSI DebugFS stream disconnected or in error state (revents=0x%x)\n",
				pfd.revents);
			break;
		}

		if (ret > 0 && (pfd.revents & (POLLIN | POLLRDNORM))) {
			/* Drain all available packets in buffer */
			while (g_running) {
				ssize_t n = read(csi_fd, &entry, sizeof(entry));
				if (n == (ssize_t)sizeof(entry)) {
					stats.total_read++;

					/* Fallback: if kernel did not populate seq_num, synthesize monotonic seq */
					static uint32_t s_local_seq = 0;
					if (entry.seq_num == 0) {
						entry.seq_num = ++s_local_seq;
					}

					/* Track sequence gaps */
					if (stats.first_pkt) {
						stats.last_seq = entry.seq_num;
						stats.first_pkt = false;
					} else {
						uint32_t expected = stats.last_seq + 1;
						if (entry.seq_num > expected) {
							stats.seq_gaps += (entry.seq_num - expected);
						}
						stats.last_seq = entry.seq_num;
					}

					/* Transmit UDP packet */
					ssize_t sent = sendto(sock_fd, &entry, sizeof(entry), 0,
							      (struct sockaddr *)&target_addr,
							      sizeof(target_addr));
					if (sent == (ssize_t)sizeof(entry)) {
						stats.total_sent++;
						stats.bytes_sent += sent;
						interval_pkts++;
						interval_bytes += sent;

						if (cfg.verbose) {
							printf("[V] Seq: %u | MAC: %02x:%02x:%02x:%02x:%02x:%02x | "
							       "RSSI: [%d, %d, %d, %d] | BW: %d | Ch: %d\n",
							       entry.seq_num,
							       entry.src_mac[0], entry.src_mac[1],
							       entry.src_mac[2], entry.src_mac[3],
							       entry.src_mac[4], entry.src_mac[5],
							       entry.rssi[0], entry.rssi[1],
							       entry.rssi[2], entry.rssi[3],
							       entry.bw, entry.channel);
						}
					} else if (sent < 0) {
						if (errno == EAGAIN || errno == EWOULDBLOCK) {
							stats.send_dropped++;
						} else {
							stats.send_errors++;
						}
					}
				} else if (n < 0) {
					if (errno == EAGAIN || errno == EWOULDBLOCK) {
						/* Ring buffer empty */
						break;
					}
					if (errno == EINTR) {
						continue;
					}
					fprintf(stderr, "Error: read() failed: %s\n",
						strerror(errno));
					break;
				} else if (n == 0) {
					/* Capture halted or EOF */
					break;
				} else {
					stats.incomplete_reads++;
					break;
				}
			}
		}

		/* Periodic stats reporting */
		if (cfg.stats_interval_s > 0 &&
		    (now_ms - last_report_ms) >= (uint64_t)(cfg.stats_interval_s * 1000)) {
			double delta_s = (double)(now_ms - last_report_ms) / 1000.0;
			double pkt_rate = (double)interval_pkts / delta_s;
			double kb_rate = (double)interval_bytes / (1024.0 * delta_s);

			printf("[CSI] Rate: %6.1f pkt/s (%6.1f KB/s) | Total: %8llu sent | Drops: %llu | Gaps: %llu\n",
			       pkt_rate, kb_rate,
			       (unsigned long long)stats.total_sent,
			       (unsigned long long)stats.send_dropped,
			       (unsigned long long)stats.seq_gaps);
			fflush(stdout);

			last_report_ms = now_ms;
			interval_pkts = 0;
			interval_bytes = 0;
		}
	}

	uint64_t end_ms = get_time_ms();
	double elapsed_s = (double)(end_ms - start_ms) / 1000.0;

	printf("\n--- CSI Extractor Termination Summary ---\n");
	printf("  Elapsed Time:       %.2f seconds\n", elapsed_s);
	printf("  Total Packets Read: %llu\n", (unsigned long long)stats.total_read);
	printf("  Total Packets Sent: %llu\n", (unsigned long long)stats.total_sent);
	printf("  Total Bytes Sent:   %llu bytes (%.2f MB)\n",
	       (unsigned long long)stats.bytes_sent,
	       (double)stats.bytes_sent / (1024.0 * 1024.0));
	printf("  Socket Drops:       %llu\n", (unsigned long long)stats.send_dropped);
	printf("  Send Errors:        %llu\n", (unsigned long long)stats.send_errors);
	printf("  Sequence Gaps:      %llu\n", (unsigned long long)stats.seq_gaps);
	printf("  Incomplete Reads:   %llu\n", (unsigned long long)stats.incomplete_reads);

	close(sock_fd);
	close(csi_fd);

	if (cfg.auto_enable) {
		printf("[+] Cleanly disabling CSI capture on %s...\n", cfg.enable_path);
		set_csi_enable(cfg.enable_path, 0);
	}

	printf("[+] Clean exit completed.\n");
	return 0;
}
