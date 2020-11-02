import serial
import cv2
import numpy as np
import time
import os
import sys


class Camera:
    def __init__(self, port='COM3', height=240, width=320):
        self.COMMAND = ['*', 'R', 'D', 'Y', '*']

        self.height = height
        self.width = width

        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = 921600
        self.serial.bytesize = serial.EIGHTBITS
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.parity = serial.PARITY_NONE
        self.serial.timeout = 2

    def connect(self):
        self.serial.open()

    def read(self):
        raw = self.serial.read()
        if (len(raw) != 1):
            return False
        return int.from_bytes(raw, "big") & 0xFF

    def isImageStart(self, index=0):
        if index < len(self.COMMAND):
            a = self.read()
            if a == False or a >= 128:
                return False
            a = chr(a)
            if self.COMMAND[index] == a:
                return self.isImageStart(index + 1)
            else:
                return False
        return True

    def getImg(self):
        index = 0
        raw = bytearray()

        COMMAND = [b'*', b'R', b'D', b'Y', b'*']

        while True:
            hi = self.serial.read()
            if index < len(COMMAND):
                if COMMAND[index] == hi:
                    if index == len(COMMAND) - 1:
                        raw = raw[:-4]
                        index = 0
                        break
                    else:
                        index = index + 1
                else:
                    index = 0

            raw += hi

        # f = open("pic_" + str(time.time()) + ".jpeg", "x+b")
        # f.write(raw)
        # f.close()

        return raw, False

    def display(self, img):
        if img is not None:
            nparr = np.frombuffer(img, np.uint8)
            frame_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            cv2.imshow("", frame_bgr)
            cv2.waitKey(1)

cam = Camera()
cam.connect()
time.sleep(2)

last = time.time()

# while not cam.isImageStart():
#     pass


while True:
    try:
        img, Err = cam.getImg()
        now = time.time()
        print("Hi", 1 / (now - last))
        last = now
        cam.display(img)
    except KeyboardInterrupt:
        sys.exit()
    except:
        pass
