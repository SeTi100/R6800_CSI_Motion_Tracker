import sys

config_path = "/home/tim/openwrt-sdk/openwrt-sdk-25.12.2-ramips-mt7621_gcc-14.3.0_musl.Linux-x86_64/.config"
with open(config_path, "r") as f:
    lines = f.readlines()

new_lines = []
disabled_count = 0
for line in lines:
    if any(k in line for k in ["hostapd", "wpad", "wolfssl", "openssl", "libtool", "libltdl", "cryptodev"]):
        if line.startswith("CONFIG_PACKAGE_") and "=" in line:
            pkg = line.split("=")[0]
            new_lines.append(f"# {pkg} is not set\n")
            disabled_count += 1
            continue
    new_lines.append(line)

with open(config_path, "w") as f:
    f.writelines(new_lines)

print(f"Successfully disabled {disabled_count} unnecessary packages in .config!")
