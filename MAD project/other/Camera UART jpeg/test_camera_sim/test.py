import msvcrt
import sys

COMMAND = [b'*', b'R', b'D', b'Y', b'*']
index = 0
raw = bytearray()

while True:
    hi = msvcrt.getch()
    if index < len(COMMAND):
        if COMMAND[index] == hi:
            if index == len(COMMAND) - 1:
                raw = raw[:-4]
                print("match", raw)
                index = 0
            else:
                index = index + 1
        else:
            index = 0

    print(hi, index)

    raw += hi

    if hi == b'\x03':
        sys.exit()