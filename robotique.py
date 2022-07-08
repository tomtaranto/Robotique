import microbit
import time
import machine
import music

microbit.i2c.init()

PI = 3.1415

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

    def rotate_and_run(self, v1, v2):
        vit1 = abs(v1) * 255 // 100
        sens1 = 1 if v1 >=0 else 2
        vit2 = abs(v2) * 255 // 100
        sens2 = 1 if v2 >=0 else 2
        microbit.i2c.write(self.addr, bytearray([0, sens1, vit1]))
        microbit.i2c.write(self.addr, bytearray([2, sens2, vit2]))



previous_error = 0


class Controler(object):
    def __init__(self, P,I,D, target):
        self.P = P
        self.I = I
        self.D = D
        self.previous_error = 10000
        self.dt = 1 / 10
        self.target = target
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
        self.previous_error = error
        return a


class Controler_double(object):
    def __init__(self, P,I,D, target1, target2):
        self.P = P
        self.I = I
        self.D = D
        self.previous_error = 10000
        self.dt = 1 / 10
        self.target1 = target1
        self.target2 = target2
        self.P_error_v = 1000
        self.I_error_v = 1000
        self.D_error_v = 1
        self.previous_error_v = 1
        self.previous_a_v = 100000
        self.P_error_theta = 1000
        self.I_error_theta = 1000
        self.D_error_theta = 1
        self.previous_error_theta = 1
        self.previous_a_theta = 100000

    def control(self, x):
        v, theta = x
        error_v = v - self.target1
        error_theta = theta - self.target2
        self.P_error_v = self.P * error_v
        self.I_error_v += self.I * error_v * self.dt
        self.D_error_v += self.D * (self.previous_error_v - error_v) * self.dt
        a = self.P_error_v + self.I_error_v + self.D_error_v

        self.P_error_theta = self.P * error_theta
        self.I_error_theta += self.I * error_theta * self.dt
        self.D_error_theta += self.D * (self.previous_error_theta - error_theta) * self.dt
        b = self.P_error_theta + self.I_error_theta + self.D_error_theta
        self.previous_error_v = error_v
        self.previous_error_theta = error_theta
        return a,b


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
L = 0.1000
dt = 1/10


def get_distance_parcourue():
    global previous_l, previous_r
    l, r = read_coder()
    diff_tours_l = l - previous_l
    diff_tours_r = r - previous_r
    distance_l = (diff_tours_l / 80) * 2 * 3.1415 * R * 100
    distance_r = (diff_tours_r / 80) * 2 * 3.1415 * R * 100
    previous_l, previous_r = l, r
    return distance_l, distance_r


def get_vitesse(d_l, d_r):
    return d_l/dt, d_r / dt

def convert(v,theta, theta_p):
    v_r = v/R - R* ((theta-theta_p)/dt)* L/2
    v_l = v/R + R* ((theta-theta_p)/dt)* L/2
    return v_l, v_r



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


distance_controler = Controler(10,0.1,0.01, 100)
angle_controler = Controler(10,0.1,0.01,0)
# vitesse_angle_controler = Controler_double(1,0.1,0.01)

rb = Robot(0x10)

welcome()

from microbit import *
import sys
# compass.calibrate()
all_d = 0
all_theta = 0
all_v = 0
all_angles = 0
threshold = 1
previous_angle = compass.heading()
while True:
    theta = angle_controler.control(all_angles)







while True:
    # sing()
    # x =  rb.distance()
    # print(rb.distance())
    print("all d", str(all_d))
    current_angle = compass.heading() #* PI / 180
    print("current angle : ", current_angle)
    u = distance_controler.control(all_d)
    theta1, theta2 = convert(u, current_angle, previous_angle)
    print("premier u", str(u))
    print("vrai U", str(u))
    if u > 100: u = 100
    if u < -100: u = -100
    if u > threshold:
        u = int(u)
        rb.setVitesse((u + 20))
        print(u + 20, "recule")
        rb.recule()
        d_l, d_r = get_distance_parcourue()
        all_d -= (d_l+d_r)/2
    elif u < -threshold:
        u = int(u)
        rb.setVitesse(-u + 20)
        print(-u + 20, "avance")
        rb.avance()
        d_l, d_r = get_distance_parcourue()
        all_d += (d_l+d_r)/2
    else:
        rb.setVitesse(0)
        rb.stop()
    print(u)
    print("----")
    # sleep(300)

sleep(3000)
goodbye()