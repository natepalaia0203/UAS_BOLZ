import serial, struct, math, sys, time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
buf = b''
t0 = time.time()
last_print = 0

def checksum_ok(pkt):
    a = b = 0
    for c in pkt[:-2]:
        a = (a + c) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b]) == pkt[-2:]

while True:
    buf += ser.read(256)
    while True:
        i = buf.find(b'\x75\x65')
        if i < 0 or len(buf) < i + 4:
            break
        plen = buf[i+3]
        total = 4 + plen + 2
        if len(buf) < i + total:
            break
        pkt = buf[i:i+total]
        buf = buf[i+total:]
        if not checksum_ok(pkt):
            continue
        dset, payload = pkt[2], pkt[4:4+plen]
        j = 0
        while j < len(payload) - 1:
            flen, fdesc = payload[j], payload[j+1]
            if flen < 2:
                break
            fdata = payload[j+2:j+flen]
            if dset == 0x80 and fdesc == 0x05 and len(fdata) >= 12:
                r, p, y = struct.unpack('>fff', fdata[:12])
                now = time.time()
                if now - last_print >= 0.5:
                    last_print = now
                    print(f"t={now-t0:6.1f}s  gz={math.degrees(y):+8.2f}")
                    sys.stdout.flush()
            j += flen
