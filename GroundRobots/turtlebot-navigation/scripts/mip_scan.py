import sys, struct
from collections import Counter

data = open('/tmp/imu_raw2.bin','rb').read()
found = Counter()
i = 0
while i < len(data) - 4:
    if data[i] == 0x75 and data[i+1] == 0x65:
        dset = data[i+2]
        plen = data[i+3]
        if i + 4 + plen + 2 > len(data):
            break
        payload = data[i+4 : i+4+plen]
        # walk fields
        j = 0
        while j < len(payload) - 1:
            flen = payload[j]
            fdesc = payload[j+1]
            if flen < 2:
                break
            found[(dset, fdesc, flen)] += 1
            j += flen
        i += 4 + plen + 2
    else:
        i += 1

print(f"{'SET':>5} {'FIELD':>7} {'LEN':>5}  COUNT")
for (dset, fdesc, flen), n in sorted(found.items()):
    print(f" 0x{dset:02X}   0x{fdesc:02X}   {flen:3d}   {n}")
