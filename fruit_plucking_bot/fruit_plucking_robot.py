from collections import deque

import numpy as np

import imutils

import cv2

import numpy as np

import time

import wiringpi as wp

import time

import serial


wp.wiringPiSetupGpio()

def motor(x, y, pwm):   #FUNCTION FOR THE MOTOR PINS AND ITS PWM

    wp.pinMode(x, 1)

    wp.pinMode(y, 1)

    wp.pinMode(pwm, 1)

    wp.softPwmCreate(pwm, 0, 100)

    return x, y, pwm

def forward(motor1, speed1, motor2, speed2):  #FUNCTION FOR THE SPEED AND DIRECTION

    (x1, y1, pwm1) = motor1

    (x2, y2, pwm2) = motor2

    wp.digitalWrite(x1, 0)

    wp.digitalWrite(y1, 1)

    wp.digitalWrite(x2, 1)

    wp.digitalWrite(y2 ,0)

    wp.softPwmWrite(pwm1, speed1)

    wp.softPwmWrite(pwm2, speed2)


motor1 = motor(18, 24, 25)    #INITIALIZING THE MOTOR1 

motor2 = motor(12, 17, 23)    #INITIALIZING THE MOTOR1

lowerBound=np.array([73, 170, 151])

upperBound=np.array([179, 255, 255])



while True:

    if(wp.digitalRead(20)==1):   # REACHING THE TREE LINE 
        forward(motor1, 0, motor2, 0)
        print("IMAGE PROCESSING")
        
        cam= cv2.VideoCapture(-1)
        kernelOpen=np.ones((5,5))
        kernelClose=np.ones((20,20))
        font=cv2.cv.InitFont(cv2.cv.CV_FONT_HERSHEY_SIMPLEX,2,0.5,0,3,1)

        print("SCANNING FOR RIPE FRUITS")  #WRIST OF THE ROBOT STARTS MOVING

        x=0
        ser=serial.Serial("/dev/ttyUSB0",9600)  
        time.sleep(5)
        ser.write('3')

        while True:
            y=ser.read()
            if(y==9):
                 break
            ret, img=cam.read()
            cb=0;   
            img=cv2.resize(img,(340,220))
    
    
            #convert BGR to HSV
            imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
            # create the Mask to hide the back ground
            mask=cv2.inRange(imgHSV,lowerBound,upperBound)
            #morphology
            maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
            maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

            maskFinal=maskClose
            conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            cb=cv2.countNonZero(maskClose)
            print(cb)

            if(cb>3000):
                        ser.write('2')
                        
            #cv2.drawContours(img,conts,-1,(255,0,0),3)
            for i in range(len(conts)):
                x,y,w,h=cv2.boundingRect(conts[i])
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255), 2)
                cv2.cv.PutText(cv2.cv.fromarray(img), str(i+1),(x,y+h),font,(0,255,255))
            cv2.imshow("maskClose",maskClose)
            cv2.imshow("maskOpen",maskOpen)
            cv2.imshow("mask",mask)
            cv2.imshow("cam",img)
            cv2.waitKey(10)
            k= cv2.waitKey(1)
            if k==27:
                break
            cam.release()
            cv2.destroyAllWindows()
            for i in range(100):
             forward(motor1, 70, motor2, 80)
    else:

            if ((wp.digitalRead(21)==1) & (wp.digitalRead(22)==1)):  #forward  

                forward(motor1, 70, motor2, 80)
                print("forward")

            elif ((wp.digitalRead(21)==0) & (wp.digitalRead(22)==1)): #right turn 
 
                forward(motor1, 0, motor2, 30)
                print("right")

            elif ((wp.digitalRead(21)==1) & (wp.digitalRead(22)==0)): #left turn
    
                forward(motor1, 30, motor2, 0)
                print("left")

            else:                                           #stay still

                forward(motor1, 0, motor2, 0)
                print("stop")






