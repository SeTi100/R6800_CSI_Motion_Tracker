import serial
import time

s = serial.Serial('COM11', 57600, timeout=3)
s.read_all()
cmd = """cat << 'EOF' > /tmp/wpa_sta.conf
ctrl_interface=/var/run/wpa_supplicant
network={
    ssid="NETGEAR49-5G"
    psk="zanystreet862"
}
EOF
cat /tmp/wpa_sta.conf
"""
s.write(cmd.encode('utf-8') + b'\n')
time.sleep(1.5)
out = s.read_all().decode('utf-8', errors='replace')
print(out)
s.close()
