#Below we are importing functionality to our Code, OPEN-CV, Time, and Pimoroni Pan Tilt Hat Package of particular note.
import cv2, sys, time, os

# Load the BCM V4l2 driver for /dev/video0. This driver has been installed from earlier terminal commands. 
#This is really just to ensure everything is as it should be.
os.system('sudo modprobe bcm2835-v4l2')
# Set the framerate (not sure this does anything! But you can change the number after | -p | to allegedly increase or decrease the framerate).
os.system('v4l2-ctl -p 40')

def rotate(servo,angle_percentage):
    ServoBlaster = open('/dev/servoblaster', 'w')   # ServoBlaster is what we use to control the servo motors
    ServoBlaster.write(f'{servo}=' + str(angle_percentage) + '%\n')   #
    ServoBlaster.flush()
rotate(0,50)
rotate(1,50)

def damped_factor(x,max_factor,smoothing): 
    #This function makes the PID proportional factor smaller near the initial position (50% of RoM)
    #It's used to dampen the rotation when the camera is near to looking directly to the target
    return ((smoothing)*x**2 + (1-smoothing))*max_factor

# Frame Size. Smaller is faster, but less accurate.
# Wide and short is better, since moving your head up and down is harder to do.
# W = 160 and H = 100 are good settings if you are using and earlier Raspberry Pi Version.
FRAME_W = 320
FRAME_H = 200

# Default Pan/Tilt for the camera in degrees. I have set it up to roughly point at my face location when it starts the code.
# Camera range is from 0 to 180. Alter the values below to determine the starting point for your pan and tilt.

# Set up the Cascade Classifier for face tracking. This is using the Haar Cascade face recognition method with LBP = Local Binary Patterns. 
# Seen below is commented out the slower method to get face tracking done using only the HAAR method.
# cascPath = 'haarcascade_frontalface_default.xml' # sys.argv[1]
cascPath = '/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml'
faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Start and set up the video capture with our selected frame size. Make sure these values match the same width and height values that you choose at the start.
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320);
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 200);
time.sleep(2)

# Turn the camera to the Start position (the data that pan() and tilt() functions expect to see are any numbers between -90 to 90 degrees).

# Light control down here. If you have a LED stick wired up to the Pimoroni HAT it will light up when it has located a face.

#Below we are creating an infinite loop, the system will run forever or until we manually tell it to stop (or use the "q" button on our keyboard)
theta_pan = 50 # initial position of pan servo
theta_tilt = 50 #initial position of tilt servo

while True:

    # Capture frame-by-frame
    ret, frame = cap.read()
    # This line lets you mount the camera the "right" way up, with neopixels above
    #frame = cv2.flip(frame, -1)
    
    if ret == False:
      print("Error getting image")
      continue

    # Convert to greyscale for easier faster accurate face detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist( gray )

    # Do face detection to search for faces from these captures frames
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))
   
    # Slower method (this gets used only if the slower HAAR method was uncommented above. 
    '''faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=4,
        minSize=(20, 20),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE | cv2.cv.CV_HAAR_FIND_BIGGEST_OBJECT | cv2.cv.CV_HAAR_DO_ROUGH_SEARCH
    )'''

    #Below draws the rectangle onto the screen then determines how to move the camera module so that the face can always be in the centre of screen. 

    for (x, y, w, h) in faces:
        # Draw a green rectangle around the face (There is a lot of control to be had here, for example If you want a bigger border change 4 to 8)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 4)

        # Track face with the square around it
        
        # Get the centre of the face
        x = x + (w/2)
        y = y + (h/2)

        # Correct relative to centre of image
        turn_x  = float(x - (FRAME_W/2))
        turn_y  = float(y - (FRAME_H/2))

        # Convert to percentage offset
        turn_x  /= float(FRAME_W/2)
        turn_y  /= float(FRAME_H/2)
        #print(turn_x)
        k_x = damped_factor(turn_x,15,.75)
        k_y = damped_factor(turn_x,15,.75)
        
        # Scale offset to degrees (that 2.5 value below acts like the Proportional factor in PID)
        turn_x   *= k_x # VFOV
        #print(turn_x)
        turn_y   *= k_y # HFOV
        cam_pan   = -turn_x
        cam_tilt  = turn_y
        ANG = [cam_pan,cam_tilt]
        servos = ["pan","tilt"]
        # Update the servos
        
        for i in range(len(servos)):
                angle = ANG[i];
                if i == 0:
                    offset_pan = (angle/180)*100 #finds RoM percentage needed
                    theta_pan = theta_pan+offset_pan #From the initial position (50% of the RoM), offsets the current servo position
                    rotate(i,theta_pan)
                else:
                    offset_tilt = (angle/180)*100 #finds RoM percentage needed
                    theta_tilt = theta_tilt+offset_tilt #From the initial position (50% of the RoM), offsets the current servo position
                    rotate(i,theta_tilt)
                print("Send angle {} to Servo {}".format(angle,servos[i]))
                time.sleep(0.01)

        break
    time.sleep(0.1)
    #Orientate the frame so you can see it.
    frame = cv2.resize(frame, (540,300))
    frame = cv2.flip(frame, 1)
   
    # Display the video captured, with rectangles overlayed
    # onto the Pi desktop 
    cv2.imshow('Video', frame)

    #If you type q at any point this will end the loop and thus end the code.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture information and stop everything
video_capture.release()
cv2.destroyAllWindows()