# Makefile for csi_extractor
# Cross-compiles for OpenWrt mipsel_24kc or compiles natively for host

TOOLCHAIN_DIR ?= /home/nos/openwrt/staging_dir/toolchain-mipsel_24kc_gcc-14.3.0_musl
CC = $(TOOLCHAIN_DIR)/bin/mipsel-openwrt-linux-gcc
export STAGING_DIR ?= /home/nos/openwrt/staging_dir

CFLAGS ?= -Os -pipe -mips32r2 -mtune=24kc -msoft-float -Wall -Wextra -Wno-unused-parameter
LDFLAGS ?= -s

TARGET = csi_extractor
SRC = csi_extractor.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean
