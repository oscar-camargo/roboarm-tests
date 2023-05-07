import cv2
import time    #https://docs.python.org/fr/3/library/time.html
import numpy as np

#thres = 0.45 # Threshold to detect object

servos = ["pan","tilt"]

classNames = []
classFile = "/home/oscar/Desktop/Object_Detection_Files/coco.names"
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath = "/home/oscar/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/oscar/Desktop/Object_Detection_Files/frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

ANG_DUMMY = np.linspace(0,1,200)

# Upper limit
_Servo1UL = 250
_Servo0UL = 230

# Lower Limit
_Servo1LL = 75
_Servo0LL = 70

def rotate(servo,angle_percentage):
    ServoBlaster = open('/dev/servoblaster', 'w')   # ServoBlaster is what we use to control the servo motors
    ServoBlaster.write(f'{servo}=' + str(angle_percentage) + '%\n')   #
    ServoBlaster.flush()
rotate(0,50)
rotate(1,50)

def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
    #print(classIds,bbox)
    if len(objects) == 0: objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box,className])
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)

    return img,objectInfo

def damped_angle(angle,smoothness):
	x = angle/90
	return ((x**smoothness)/(x**smoothness+(1-x)**smoothness))*90

if __name__ == "__main__":

    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    #cap.set(10,70)

    theta = 0.5 #Initial servo position at 50% of RoM
    
    while True:
        success, img = cap.read()
        result, objectInfo = getObjects(img,0.2,0.2, objects=['sports ball'])
        
        if len(objectInfo) > 0:
            box_x_center = objectInfo[0][0][0] + int((objectInfo[0][0][2])/2)
            box_y_center = objectInfo[0][0][1] + int((objectInfo[0][0][3])/2)
            cv2.drawMarker(img,(box_x_center,box_y_center),color=(0,255,255),markerType = 0,markerSize = 10,thickness = 2)
            cv2.drawMarker(img,(320,240),color=(0,0,255),markerType=1,thickness=3)
        # Correct relative to centre of image
            turn_x  = float(box_x_center - (640/2))
            turn_y  = float(box_y_center - (480/2))

            # Convert to percentage offset
            turn_x  /= float(640)
            turn_y  /= float(480)

            # Scale offset to degrees (that 2.5 value below acts like the Proportional factor in PID)
            if .4 < turn_x < .6:
                turn_x   *= 62*.2 # VFOV
                turn_y   *= 49*.5 # HFOV
            else:
                turn_x   *= 62*.8 # VFOV
                turn_y   *= 49*.5 # HFOV
            cam_pan   = -turn_x
            cam_tilt  = turn_y

            # Clamp Pan/Tilt to 0 to 180 degrees
            #cam_pan = max(0,min(180,cam_pan))
            #cam_tilt = max(0,min(180,cam_tilt))
            ANG = [cam_pan,cam_tilt]
            # Update the servos
            #pan(int(cam_pan-90))
            #tilt(int(cam_tilt-90))
            for i in range(len(servos)-1):
                angle = ANG[i];
                offset = (angle/180)*100 #finds RoM percentage needed
                theta = theta+offset #From the initial position (50% of the RoM), offsets the current servo position
                print(theta)
                print("Send angle {} to Servo {}".format(angle,servos[i]))
                rotate(i,theta)
                time.sleep(0.01)
        cv2.imshow("Output",img)
        cv2.waitKey(1)
