import serial
import cv2
import numpy as np
import time
import os
import sys


class Camera:
    def __init__(self, port='COM3'):
        self.COMMAND = b'*RDY*'

        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = 2250000
        self.serial.bytesize = serial.EIGHTBITS
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.parity = serial.PARITY_NONE
        self.serial.timeout = 2

        self.remain = bytearray()

    def connect(self):
        self.serial.open()

    def getImg(self):
        raw = bytearray(self.remain)
        rx = 0

        while True:
            bytesToRead = self.serial.inWaiting()
            raw += self.serial.read(bytesToRead)

            find = raw.find(self.COMMAND, min(rx-5, 0))
            if find != -1 :
                self.remain = raw[find+5:]
                raw = raw[:find]
                break
            rx += bytesToRead

        # f = open("pic_" + str(time.time()) + ".jpeg", "x+b")
        # f.write(raw)
        # f.close()

        return raw

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

while True:
    try:
        img = cam.getImg()
        now = time.time()
        print("{:.5f}".format(1 / (now - last)), "fps", len(img))
        last = now
        cam.display(img)
    except KeyboardInterrupt:
        sys.exit()
    except Exception as e:
        print(e)
