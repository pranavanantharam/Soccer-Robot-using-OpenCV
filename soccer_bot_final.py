import time
import wiringpi as wp
import numpy as np
import imutils
import cv2

x=0                                                             
y=0
r=0

HSV_Lower_White = (7,15,240)
HSV_Upper_White = (30,0,255)

wp.wiringPiSetupGpio()                                         #WiringPi setup and all gpio pins initialized 

m1x=26
m1y=20
m2x=19
m2y=16
m1e=13
m2e=21

wp.pinMode(m1x,1)                                              #Motor 1 and Motor 2 - Forward,Backward and Enabler pin Setup
wp.pinMode(m1y,1)
wp.softPwmCreate(m1e,0,100)
wp.pinMode(m2x,1)
wp.pinMode(m2y,1)
wp.softPwmCreate(m2e,0,100)

def forward(dist):
        wp.digitalWrite(m1x,0)
        wp.digitalWrite(m1y,1)
        wp.digitalWrite(m2x,1)
        wp.digitalWrite(m2y,0)

        wp.softPwmWrite(m1e,20)
        wp.softPwmWrite(m2e,25)

        print('Moving forward')
        time.sleep(dist)
        wp.softPwmWrite(m1e,0)
        wp.softPwmWrite(m2e,0)

def reverse(dist):
        wp.digitalWrite(m1x,1)
        wp.digitalWrite(m1y,0)
        wp.digitalWrite(m2x,0)
        wp.digitalWrite(m2y,1)

        wp.softPwmWrite(m1e,20)
        wp.softPwmWrite(m2e,25)

 		print('Moving backward')
        time.sleep(dist)
        wp.softPwmWrite(m1e,0)
        wp.softPwmWrite(m2e,0)

def left_turn(deg):
        wp.digitalWrite(m1x,0)
        wp.digitalWrite(m1y,1)
        wp.digitalWrite(m2x,0)
        wp.digitalWrite(m2y,1)
        
        wp.softPwmWrite(m1e,20)
        wp.softPwmWrite(m2e,20)

        print('Turning left')
        time.sleep(deg)
        wp.softPwmWrite(m1e,0)
        wp.softPwmWrite(m2e,0)


def right_turn(deg):
        wp.digitalWrite(m1x,1)
        wp.digitalWrite(m1y,0)
        wp.digitalWrite(m2x,1)
        wp.digitalWrite(m2y,0)
        
        wp.softPwmWrite(m1e,20)
        wp.softPwmWrite(m2e,20)
       
        print('Turning right')
        time.sleep(deg)
        wp.softPwmWrite(m1e,0)
        wp.softPwmWrite(m2e,0)

def burst_movement(dist):                                               #Goal scorer function
	wp.digitalWrite(m1x,0)
    wp.digitalWrite(m1y,1)
    wp.digitalWrite(m2x,1)
    wp.digitalWrite(m2y,0)

    wp.softPwmWrite(m1e,100)
    wp.softPwmWrite(m2e,100)

    time.sleep(dist)
    wp.softPwmWrite(m1e,0)
    wp.softPwmWrite(m2e,0)   

    wp.digitalWrite(m1x,1)
    wp.digitalWrite(m1y,0)
    wp.digitalWrite(m2x,0)
    wp.digitalWrite(m2y,1)

    wp.softPwmWrite(m1e,100)
    wp.softPwmWrite(m2e,100)

    time.sleep(dist)
    wp.softPwmWrite(m1e,0)
    wp.softPwmWrite(m2e,0)  


def ball_tracker(success):                                     #success is a flag to check whether robot has reached the ball
      
	while (True and success!=1):
        for i in range (0,5):
            camera.grab()
        ret, frame = camera.read()                             # grab the current frame
        kernel=np.ones((5,5),np.uint8)

        frame = imutils.resize(frame,width=600,height=400)     # resize the frame to size 600X400
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)         # blur the frame to smoothen it,
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)           # convert it to the HSV color space
        mask = cv2.inRange(hsv, HSV_Lower_Orange, HSV_Upper_Orange)          # construct a mask for the separating color from background          
        mask = cv2.erode(mask, None, iterations=2)             #Perform a series of dilations and erosions to increase quality
        mask = cv2.dilate(mask, None, iterations=2)  
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)      #Morphological transformations
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None                                          #initialize the center of ball (x,y) to none

        if len(cnts) > 0:                                      # only proceed if at least one contour was found
                c = max(cnts, key=cv2.contourArea)             # find the largest contour in the mask
                ((x, y), radius) = cv2.minEnclosingCircle(c)   # it to compute the minimum enclosing circle and
                M = cv2.moments(c)                             # find the centroid
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                x=int(x)
                y=int(y)
                r=int(radius)
                if (radius > 0 and radius < 115):                # only proceed if the radius meets a minimum size        
                        print x,y,r
                        output = "X{0:d}Y{1:d}Z{2:d}".format(x,y,r)
                        forward(0.5)                           #ball is close enough that radius exceeds upper limit
                elif radius>115:                                
                        print x,y,r
                        output = "X{0:d}Y{1:d}Z{2:d}".format(x,y,r)
                        forward(1.3)
                        success=1                               #ball located
        else:   
                right_turn(0.3)



def goal_tracker(final_success):                                #locates goal                  
    sensitivity=75
    lower_white=np.array([0,0,255-sensitivity])
    upper_white=np.array([255,sensitivity,255])
	while (True and final_success!=1):
        for i in range (0,5):
            camera.grab()
        ret, frame = camera.read()                             
        kernel=np.ones((5,5),np.uint8)

        frame = imutils.resize(frame,width=600,height=400)     
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)         
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)           
        mask = cv2.inRange(hsv, HSV_Lower_White, HSV_Upper_White)                  
        mask = cv2.erode(mask, None, iterations=2)             
        mask = cv2.dilate(mask, None, iterations=2)  
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
        if len(cnts) >  0:                
            goal = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(goal)
            if (area > 0 and area < 300):                       
                print area
                forward(0.5)
            elif area > 300:
                print area
                burst_movement(1)
                final_success=1
        else:
                right_turn(0.3)    

                       



camera = cv2.VideoCapture(0)                                   #initialize the camera
success=0
final_success=0
ball_tracker(success)
print "Success! Ball has been located"
print "Tracking goal"
goal_tracker(final_success)
print "GOAL!"
camera.release()                                                #release the camera
