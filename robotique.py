import microbit
import time
import machine
import music

microbit.i2c.init()


class Robot():
    def __init__(self, addr=0x10):
        """Initiaisation robot
        addr : adresse i2c. 0x10 par defaut"""
        self.addr = addr
        self._vitesse = 0  # vitesse entre 0 et 100

    def getVitesse(self):
        return self._vitesse

    def setVitesse(self, v):
        self._vitesse = v

    def moteurDroit(self, v=None):
        if v == None:
            v = self._vitesse
        sens = 0
        if v >= 0:
            sens = 1
        else:
            sens = 2
        vit = abs(v) * 255 // 100  # vitesse moteur 0..255
        microbit.i2c.write(self.addr, bytearray([0, sens, vit]))

    def moteurGauche(self, v=None):
        if v == None:
            v = self._vitesse
        sens = 0
        if v >= 0:
            sens = 1
        else:
            sens = 2
        vit = abs(v) * 255 // 100  # vitesse moteur 0..255
        microbit.i2c.write(self.addr, bytearray([2, sens, vit]))

    def avance(self, v=None):
        if v != None:
            self._vitesse = v
        self.moteurDroit()
        self.moteurGauche()

    def recule(self, v=None):
        if v is not None:
            self._vitesse = v
        self.moteurDroit(-self._vitesse)
        self.moteurGauche(-self._vitesse)

    def stop(self):
        microbit.i2c.write(self.addr, bytearray([0, 2, 0]))
        microbit.sleep(1)
        microbit.i2c.write(self.addr, bytearray([2, 2, 0]))

    def distance(self):
        """Calcule la distance Ã  l'obstacle en cm
        pin1 : Trig
        pin2 : Echo"""
        microbit.pin1.write_digital(1)
        time.sleep_ms(10)
        microbit.pin1.write_digital(0)
        microbit.pin2.read_digital()
        t2 = machine.time_pulse_us(microbit.pin2, 1)
        d = 340 * t2 / 20000
        return d

    def son_r2d2(self):
        tune = ["A7:0", "G7:0", "E7:0", "C7:0", "D7:0", "B7:0", "F7:0", "C8:0", "A7:0", "G7:0", "E7:0", "C7:0", "D7:0",
                "B7:0", "F7:0", "C8:0"]
        music.play(tune)

    def son_bip(self):
        for i in range(2):
            freq = 2000
            while freq > 1000:
                music.pitch(int(freq), 10)
                freq *= 0.95
            freq = 1000
            while freq < 3000:
                music.pitch(int(freq), 10)
                freq *= 1.05

    def tourner_droite(self, v=None):
        if v == None:
            v = self._vitesse
        vit = abs(v) * 255 // 100
        microbit.i2c.write(self.addr, bytearray([0, 1, vit]))
        microbit.i2c.write(self.addr, bytearray([2, 2, vit]))

    def tourner_gauche(self, v=None):
        if v == None:
            v = self._vitesse
        vit = abs(v) * 255 // 100
        microbit.i2c.write(self.addr, bytearray([0, 2, vit]))
        microbit.i2c.write(self.addr, bytearray([2, 1, vit]))


previous_error = 0


class Controler(object):
    def __init__(self):
        self.P = 10
        self.I = 0.1
        self.D = 0.01
        self.previous_error = 10000
        self.dt = 1 / 10
        self.target = 100
        self.P_error = 1000
        self.I_error = 1000
        self.D_error = 1
        self.previous_a = 100000

    def control(self, x):
        error = x - self.target
        self.P_error = self.P * error
        self.I_error += self.I * error * self.dt
        self.D_error += self.D * (self.previous_error - error) * self.dt
        a = self.P_error + self.I_error + self.D_error
        # a+= self.D * (self.previous_error - error) / self.dt
        # res = a - self.previous_a
        # self.previous_a = a
        self.previous_error = error
        return a


def modelisation(theta1, theta2):
    R = 0.0215
    L = 0.1
    V = R / 2 * (theta1 + theta2)
    w = (R / L) * (theta1 - theta2)
    vx = v * cos(w)


def read_coder():
    buf = bytearray(1)
    buf[0] = 0x04
    microbit.i2c.write(0x10, buf)
    etatByteL = (microbit.i2c.read(0x10, 2))
    L = int.from_bytes(etatByteL, "big")
    buf[0] = 0x06
    microbit.i2c.write(0x10, buf)
    etatByteR = (microbit.i2c.read(0x10, 2))
    R = int.from_bytes(etatByteR, "big")
    return L, R


previous_l, previous_r = read_coder()
R = 0.0215


def get_distance_parcourue():
    global previous_l, previous_r
    l, r = read_coder()
    diff_tours_l = l - previous_l
    distance_l = (diff_tours_l / 80) * 2 * 3.1415 * R * 100
    previous_l, previous_r = l, r
    return distance_l


import music


def sing():
    tune = [
        "A:2", "C:2", "D:2", "D:2", "D", "E", "F:2", "F:2",
        "F", "G:2", "E:2", "E:2", "D:2", "C:2", "C:2", "D:2",
        "A:2", "C:2", "D:2", "D:2", "D", "E:2", "F:2", "F:2",
        "F", "G:3", "E:2", "E:2", "D", "C:2", "D",
        "A:2", "C:2", "D:2", "D:3", "D", "F", "G:2", "G",
    ]
    music.play(tune)


# sing()

def welcome():
    # microbit.display.scroll("WELCOME ! ")
    microbit.display.show(microbit.Image.SKULL)
    microbit.i2c.write(0x10, bytearray([0x0B, 1]))
    microbit.i2c.write(0x10, bytearray([0x0C, 1]))


def goodbye():
    # microbit.display.scroll("WELCOME ! ")
    microbit.display.show(microbit.Image.HEART)
    microbit.i2c.write(0x10, bytearray([0x0B, 1]))
    microbit.i2c.write(0x10, bytearray([0x0C, 1]))


controler = Controler()

rb = Robot(0x10)

welcome()

from microbit import *
import sys

all_d = 0
threshold = 1
while True:
    # sing()
    # x =  rb.distance()
    # print(rb.distance())
    print("all d", str(all_d))
    u = controler.control(all_d)
    print("premier u", str(u))
    print("vrai U", str(u))
    if u > 100: u = 100
    if u < -100: u = -100
    if u > threshold:
        u = int(u)
        rb.setVitesse((u + 20))
        print(u + 20, "recule")
        rb.recule()
        d = get_distance_parcourue()
        all_d -= d
    elif u < -threshold:
        u = int(u)
        rb.setVitesse(-u + 20)
        print(-u + 20, "avance")
        rb.avance()
        d = get_distance_parcourue()
        all_d += d
    else:
        rb.setVitesse(0)
        rb.stop()
    print(u)
    print("----")
    # sleep(300)

sleep(3000)
goodbye()