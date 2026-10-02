import sys
import serial
import time

def run_serial(cmd, wait=1.5):
    s = serial.Serial('COM11', 57600, timeout=wait + 2)
    s.read_all()
    s.write(cmd.encode('utf-8') + b'\n')
    time.sleep(wait)
    out = s.read_all().decode('utf-8', errors='replace')
    s.close()
    return out

if __name__ == '__main__':
    wait = 1.5
    cmd = 'uptime'
    if len(sys.argv) > 1:
        if sys.argv[1] == '-w' and len(sys.argv) > 3:
            wait = float(sys.argv[2])
            cmd = ' '.join(sys.argv[3:])
        else:
            cmd = ' '.join(sys.argv[1:])
    print(run_serial(cmd, wait))
