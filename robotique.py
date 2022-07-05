import microbit
import time
import machine
import music

microbit.i2c.init()

class Robot():
    def __init__(self,addr=0x10):
        """Initiaisation robot
        addr : adresse i2c. 0x10 par defaut"""
        self.addr=addr
        self._vitesse=0 # vitesse entre 0 et 100
    
    def getVitesse(self):
        return self._vitesse
        
    def setVitesse(self, v):
        self._vitesse=v
        
    def moteurDroit(self, v=None):
        if v==None:
            v=self._vitesse
        sens=0
        if v >= 0:
            sens=1
        else:
            sens=2
        vit=abs(v)*255//100   # vitesse moteur 0..255
        microbit.i2c.write(self.addr,bytearray([0,sens, vit]))
        
    def moteurGauche(self, v=None):
        if v==None:
            v=self._vitesse
        sens=0
        if v >= 0:
            sens=1
        else:
            sens=2
        vit=abs(v)*255//100   # vitesse moteur 0..255
        microbit.i2c.write(self.addr,bytearray([2,sens, vit]))

    def avance(self,v=None):
        if v != None:
            self._vitesse=v
        self.moteurDroit()
        self.moteurGauche()
        
    def recule(self, v=None):
        if v is not None:
            self._vitesse = v
        self.moteurDroit(-self._vitesse)
        self.moteurGauche(-self._vitesse)
        
    def stop(self):
        microbit.i2c.write(self.addr,bytearray([0,2,0]))
        microbit.sleep(1)
        microbit.i2c.write(self.addr,bytearray([2,2,0]))
        
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
        tune=["A7:0", "G7:0", "E7:0","C7:0","D7:0","B7:0","F7:0","C8:0","A7:0","G7:0","E7:0","C7:0","D7:0","B7:0","F7:0","C8:0"]
        music.play(tune)
        
    def son_bip(self):
        for i in range(2):
            freq=2000
            while freq>1000:
                music.pitch(int(freq),10)
                freq*=0.95
            freq=1000
            while freq<3000:
                music.pitch(int(freq),10)
                freq*=1.05
    def tourner_droite(self, v=None):
        if v==None:
            v=self._vitesse
        vit=abs(v)*255//100
        microbit.i2c.write(self.addr,bytearray([0,1, vit]))
        microbit.i2c.write(self.addr,bytearray([2,2, vit]))
    
    def tourner_gauche(self, v=None):
        if v==None:
            v=self._vitesse
        vit=abs(v)*255//100
        microbit.i2c.write(self.addr,bytearray([0,2, vit]))
        microbit.i2c.write(self.addr,bytearray([2,1, vit]))
                
            

previous_error = 0 

class Controler(object):
    def __init__(self):
        self.P=0.5
        self.I=0.1
        self.D=0.01
        self.previous_error=100
        self.dt = 1/10
        self.target = 25
    
    def control(self, x):
        a = 0
        error = x - self.target 
        a += self.P * error
        a += self.I * error * self.dt
        a+= self.D * (self.previous_error - error) / self.dt
        self.previous_error = error
        return a

    

controler = Controler()
        
rb = Robot(0x10)

from microbit import *

while True:
    x =  rb.distance()
    print(rb.distance())
    u = controler.control(x)
    u = -8 * int(u)
    if u >100 : u=100
    if u < -100 : u = -100
    if u >0:
        rb.setVitesse((u+15))
        print(u+15, "recule")
        rb.recule()
    elif u <0: 
        rb.setVitesse(-u +15)
        print(-u+15, "avance")   
        rb.avance()
    else:
        rb.setVitesse(0)
        rb.stop()
    print(u)   
    print("----")
    sleep(10)
    # sleep(300)

while True:
    rb.setVitesse(15)
    rb.recule()
 