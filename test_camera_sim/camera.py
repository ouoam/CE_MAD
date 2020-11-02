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
        self.serial.baudrate = 2250000
        self.serial.bytesize = serial.EIGHTBITS
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.parity = serial.PARITY_NONE
        self.serial.timeout = 2

    def connect(self):
        self.serial.open()
#        while not self.isImageStart():
#            pass
#        print("finish init camera")
#        cv2.startWindowThread()

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
        # self.serial.write([ord('A')])
        # print("start")
        i = 0
        frame_raw = bytearray()
        while i < self.height * self.width * 2:
            raw = self.serial.read(400)
            frame_raw += raw
            i += 400
        #frame_raw = self.serial.read(self.height * self.width * 2)

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

        Y1 = frame_raw[0::4]
        U  = frame_raw[1::4]
        Y2 = frame_raw[2::4]
        V  = frame_raw[3::4]
        
        UV = np.empty((self.height*self.width), dtype=np.uint8)
        YY = np.empty((self.height*self.width), dtype=np.uint8)
        
        UV[0::2] = np.frombuffer(U,  dtype=np.uint8)
        UV[1::2] = np.frombuffer(V,  dtype=np.uint8)
        YY[0::2] = np.frombuffer(Y1, dtype=np.uint8)
        YY[1::2] = np.frombuffer(Y2, dtype=np.uint8)
        
        UV = UV.reshape((self.height, self.width))
        YY = YY.reshape((self.height, self.width))
        
        frame_uyvy = cv2.merge([UV, YY])

        return frame_uyvy, False

    def display(self, img):
        if img is not None:
            frame_bgr  = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_UYVY)
            cv2.imshow("", frame_bgr)
            cv2.waitKey(1)

    def save(self, img, saveat):
        if img is not None:
            cv2.imwrite(saveat, img)

cam = Camera()
cam.connect()
time.sleep(2)

last = time.time()

while not cam.isImageStart():
    pass

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
