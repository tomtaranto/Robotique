# (c) 2021 Christophe Gueneau
# Pour robot MaqueenPlus

from microbit import *
from time import sleep_ms
from machine import time_pulse_us

import math


class Robot:
    def __init__(self):
        self.moteur(0, 0, 0, 0)

        self.x, self.y, self.t, self.v = 0, 0, 0, 0
        self.cGI, self.cDI = self.codeursInit()
        self.pcGI, self.pcDI = self.cGI, self.cDI
        self.cG, self.cD = 0, 0
        self.pcG, self.pcD = 0, 0

        self.R = 0.0215
        self.L = 0.1

        self.prevE, self.iE = 0, 0

    def init(self):
        i2c.init()
        sleep(500)

    def moteur(self, dirG, vitG, dirD, vitD):
        buf = bytearray(5)
        buf[0] = 0x00
        buf[1] = dirG
        buf[2] = vitG
        buf[3] = dirD
        buf[4] = vitD
        i2c.write(0x10, buf)

    def moteurG(self, dir, vit):
        buf = bytearray(3)
        buf[0] = 0x00
        buf[1] = dir
        buf[2] = vit
        i2c.write(0x10, buf)

    def moteurD(self, dir, vit):
        buf = bytearray(3)
        buf[0] = 0x02
        buf[1] = dir
        buf[2] = vit
        i2c.write(0x10, buf)

    def moteurStop(self):
        buf = bytearray(2)
        buf[0] = 0x00
        buf[1] = 0
        i2c.write(0x10, buf)
        buf[0] = 0x02
        buf[1] = 0
        i2c.write(0x10, buf)

    def ligne(self, indexCapteur):
        buf = bytearray(1)
        buf[0] = 0x1D
        i2c.write(0x10, buf)
        etatByte = (i2c.read(0x10, 1)[0])  # on récupère l'octet d'index 0 dans le tableau
        etatCapteur = etatByte & 2 ** (indexCapteur - 1)  # masque
        if etatCapteur != 0:
            etatCapteur = 1
        return etatCapteur

    def servo(self, index, angle):
        buf = bytearray(2)
        if (index == 1):
            buf[0] = 0x14
        if (index == 2):
            buf[0] = 0x15
        if (index == 3):
            buf[0] = 0x16
        buf[1] = angle
        i2c.write(0x10, buf)

    def ledRgb(self, couleurG, couleurD):
        buf = bytearray(3)
        buf[0] = 0x0b
        buf[1] = couleurG
        buf[2] = couleurD
        i2c.write(0x10, buf)

    def distance(self, pinTrig, pinEcho):
        pinTrig.write_digital(1)
        sleep_ms(10)
        pinTrig.write_digital(0)
        t = time_pulse_us(pinEcho, 1)
        d = 340 * t // 20000
        return d

    def codeursInit(self):
        try:
            i2c.write(0x10, bytearray([0x04]))
            buf = i2c.read(0x10, 4)

            a = int.from_bytes(buf[:2], 'big')
            b = int.from_bytes(buf[2:], 'big')
            return (a, b)
        except Exception as e:
            print(e)
        return (0, 0)

    def codeurs(self):
        try:
            i2c.write(0x10, bytearray([0x00]))
            buf = i2c.read(0x10, 3)

            asign = buf[0]
            if asign == 2:
                asign = -1

            bsign = buf[2]
            if bsign == 2:
                bsign = -1

            i2c.write(0x10, bytearray([0x04]))
            buf = i2c.read(0x10, 4)

            self.cGI = int.from_bytes(buf[:2], 'big')
            a = asign * (self.cGI - self.pcGI)
            self.cG += a
            self.pcGI = self.cGI

            self.cDI = int.from_bytes(buf[2:], 'big')
            b = bsign * (self.cDI - self.pcDI)
            self.cD += b
            self.pcDI = self.cDI

            return a, b

        except Exception as e:
            print(e)
        return 0, 0

    def update(self, dt):
        G, D = self.codeurs()
        dG = G / 80 * math.pi
        dD = D / 80 * math.pi
        self.t += self.R / self.L * (dD - dG)
        self.v = self.R / 2 * (dD + dG) / dt
        self.x = self.x + self.v * math.cos(self.t) * dt
        self.y = self.y + self.v * math.sin(self.t) * dt

    def move(self, v, w):
        stop_limit = 30

        G = v / self.R - self.L / 2 * w / self.R
        G = int(G / math.pi)

        dirG = 0
        if G > stop_limit:
            dirG = 1
        elif G < -stop_limit:
            dirG = 2

        G = abs(G)
        if G > 255:
            G = 255

        D = v / self.R + self.L / 2 * w / self.R
        D = int(D / math.pi)
        dirD = 0
        if D > stop_limit:
            dirD = 1
        elif D < -stop_limit:
            dirD = 2

        D = abs(D)
        if D > 255:
            D = 255

        self.moteur(dirG, G, dirD, D)
        return (dirG + dirD) > 0

    def control(self, v, t, dt):
        et = t - self.t
        self.iE += et * dt
        P = 400
        D = 1
        I = 0.01
        pid = P * et + D * (et - self.prevE) / dt  # +I*self.iE
        self.prevE = et

        return self.move(v, pid)

