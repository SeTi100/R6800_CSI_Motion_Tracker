import serial
import time

s = serial.Serial('COM11', 57600, timeout=5)
s.write(b"iw dev phy3-ap0 scan freq 5180 | grep -e BSS -e SSID -e signal\n")
time.sleep(3)
out = s.read_all().decode('utf-8', errors='replace')
print(out)
s.close()
