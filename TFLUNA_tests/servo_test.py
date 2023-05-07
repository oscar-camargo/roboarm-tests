#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
import numpy as np

#Parameters
servos = ["yaw","pitch"]
MIN_IMP  =[500,500]
MAX_IMP  =[15000,5000]
ANG_DUMMY = np.linspace(0,1,50)
ANG = [10,20]

servos_camera = ["pan","tilt"]
MIN_IMP_CAMERA = [500,500]
MAX_IMP_CAMERA = [2000,2000]
ANG_CAMERA = [0,0]

#Objects
pca = ServoKit(channels=16)

#function for damping the signals

#def damped_angle(angle,smoothness):#
#	x = angle/90##
#	return ((x**smoothness)/(x**smoothness+(1-x)**smoothness))*90


# function init 
def init():
    for i in range(0,len(servos)):
        pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])
# function main 
def main():
    pcaScenario();
# function pcaScenario 
def pcaScenario():
    #Scenario to test servo
    for i in range(len(servos)-1):
        angle = ANG[i];
        if angle != 0:
            #pca.servo[i].angle = 45
            for j in range(angle):
                print("Send angle {} to Servo {}".format(j,servos[i]))
                pca.servo[i].angle = j
                time.sleep(0.01)
            for j in range(angle,0,-1):
                print("Send angle {} to Servo {}".format(j,servos[i]))
                pca.servo[i].angle = j
                time.sleep(0.01)
            pca.servo[i].angle=None #disable channel
            time.sleep(0.5)

if __name__ == '__main__':
    init()
    main()


