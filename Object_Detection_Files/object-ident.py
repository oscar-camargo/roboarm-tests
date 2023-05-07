import cv2

#thres = 0.45 # Threshold to detect object

classNames = []
classFile = "/home/oscar/Desktop/Object_Detection_Files/coco.names"
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath = "/home/oscar/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/oscar/Desktop/Object_Detection_Files/frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(640,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
    #print(classIds,bbox)
    if len(objects) == 0: objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            pan_angle = 0
            tilt_angle = 0
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box,className])
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    box_x_center = int(box[0] + (box[2]/2))
                    box_y_center = int(box[1] + (box[3]/2))
                    cv2.drawMarker(img,(box_x_center,box_y_center),color=(0,255,255),markerType = 0,markerSize = 10,thickness = 2)
                    cv2.drawMarker(img,(320,240),color=(0,0,255),markerType=1,thickness=3)
                    
                        
                    #while box_x_center - 320 > 5:
                    #    if -90 < pan_angle < 90:
                    #        pan_angle = pan_angle - 1
                    #        print(f"adjusting camera.. Pan Angle: {pan_angle}")
                    #while box_x_center - 320 < -5:
                    #    if -90 < pan_angle < 90:
                    #        pan_angle = pan_angle + 1
                    #        print(f"adjusting camera.. Pan Angle: {pan_angle}")
                    #while box_y_center - 240 > 5:
                    #    if -90 < tilt_angle < 90:
                    #        tilt_angle = tilt_angle - 1
                    #        print(f"adjusting camera.. Tilt Angle: {tilt_angle}")
                    #while box_x_center - 240 < 5:
                    #    if -90 < tilt_angle < 90:
                    #        tilt_angle = tilt_angle + 1
                    #        print(f"adjusting camera.. Tilt Angle: {tilt_angle}")    
    return img,objectInfo


if __name__ == "__main__":

    cap = cv2.VideoCapture(0)
    cap.set(3,320)
    cap.set(4,240)
    #cap.set(10,70)


    while True:
        success, img = cap.read()
        result, objectInfo = getObjects(img,0.45,0.2,objects=['sports ball'])
        #print(objectInfo)
        cv2.imshow("Output",img)
        cv2.waitKey(1)
