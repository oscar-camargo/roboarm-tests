from multiprocessing import Process, Queue
import time
#import cv2

# Upper limit
_Servo1UL = 250
_Servo0UL = 230

# Lower Limit
_Servo1LL = 75
_Servo0LL = 70

angles = [40,-30,50,-60]

def rotate(servo,angle_percentage):
    ServoBlaster = open('/dev/servoblaster', 'w')   # ServoBlaster is what we use to control the servo motors
    ServoBlaster.write(f'{servo}=' + str(angle_percentage) + '%\n')   #
    ServoBlaster.flush()

rotate(1,40)
time.sleep(1)
#rotate(1,50)
#time.sleep(1)
#for angle in angles:
#    offset = (angle/180)*100 #finds RoM percentage needed
#    theta = 50+offset #From the initial position (50% of the RoM), offsets the current servo position
#    rotate(0,theta)
#    time.sleep(1)
#rotate(0,50)