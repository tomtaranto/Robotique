#!/usr/bin/env python3

from microbit import *
import radio
import struct

class radio_receiver:
    def __init__(self):
        radio.on()
        radio.config(length=251)
        
        self.positions = []
        
    def parse_poses(self,msg):
        self.positions = []
        a_msg = []
        a,b,c,a2,b2,c2,ball0,ball1 = 0,0,0,0,0,0,0,0
        if msg is not None:
            a_msg=msg.split(',')
        else:
            return (0,0,0,0,0,0,0,0)
        if msg is not None and len(a_msg) == 8:
            try:
                a = int(a_msg[0])
                b = int(a_msg[1])
                c = float(a_msg[2])
    
                a2 = int(a_msg[3])
                b2 = int(a_msg[4])
                c2 = float(a_msg[5])

                ball0 = int(a_msg[6])
                ball1 = int(a_msg[7])
            except Exception as e:
                return (0,0,0,0,0,0,0,0)
        return (a,b,c,a2,b2,c2,ball0,ball1)
        
    def update(self):
        return self.parse_poses(radio.receive())
