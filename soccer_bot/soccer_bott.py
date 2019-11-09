from collections import deque

import numpy as np

import imutils

import cv2

import time

import wiringpi as wp

import time



wp.wiringPiSetupGpio()





def Servo(pin):

    wp.pinMode(pin, 1)

    wp.softPwmCreate(pin, 0, 100)

    return (pin)





def Position(servo, pos):

    (pin) = servo

    wp.softPwmWrite(pin, pos)





def Sweep(servo, delay):

    (pin) = servo

    for i in range(0, 20, 1):

        wp.softPwmWrite(pin, i)

        wp.delay(delay)





def motor(x, y, pwm):

    wp.pinMode(x, 1)

    wp.pinMode(y, 1)

    wp.pinMode(pwm, 1)

    wp.softPwmCreate(pwm, 0, 100)

    return x, y, pwm





def forward(motor1, speed1, motor2, speed2):

    (x1, y1, pwm1) = motor1

    (x2, y2, pwm2) = motor2

    wp.digitalWrite(x1, 0)

    wp.digitalWrite(y1, 1)

    wp.digitalWrite(x2, 0)

    wp.digitalWrite(y2, 1)

    wp.softPwmWrite(pwm1, speed1)

    wp.softPwmWrite(pwm2, speed2)





servo = Servo(22)



motor1 = motor(18, 24, 25)

motor2 = motor(12, 17, 23)



HSV_Lower = (10, 116, 210)

HSV_Upper = (79, 255, 255)

pink_lower = (125, 109, 123)

pink_upper = (175, 255, 255)



# for i in range(0,100):

#    camera = cv2.VideoCapture(i)

#    ret,frame=camera.read()

#    if ret:

#        break



camera = cv2.VideoCapture(-1)



while True:

    (grabbed, frame) = camera.read()

    frame = imutils.resize(frame, width=900)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)



    mask = cv2.inRange(hsv, HSV_Lower, HSV_Upper)

    mask = cv2.erode(mask, None, iterations=2)

    mask = cv2.dilate(mask, None, iterations=2)



    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,

                        cv2.CHAIN_APPROX_SIMPLE)[-2]

    center = None



    if len(cnts) > 0:

        c = max(cnts, key=cv2.contourArea)

        ((x, y), radius) = cv2.minEnclosingCircle(c)

        M = cv2.moments(c)

        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))



    if radius < 10:

        x = 450

    if radius > 10:

        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

        cv2.circle(frame, center, 5, (0, 0, 255), -1)

        a = int(x)

        b = int(y)

        r = int(radius)

        print "x=", a, "y=", b, "z=", r

        key = cv2.waitKey(1) & 0xFF



        if x > 0 and x < 300:

            forward(motor1, 200, motor2, 0)

        elif x > 300 and x < 600:

            forward(motor1, 70, motor2, 70)
    
        elif x > 600 and x < 900:

            forward(motor1, 0, motor2, 20)

        else:

            forward(motor1, 70, motor2, 70)
    


    if radius > 200:

        forward(motor1, 0, motor2, 0)

        Position(servo, 15)

        wp.delay(2000)

        # Sweep(servo,100)

        break



while True:

    (grabbed, frame) = camera.read()

    frame = imutils.resize(frame, width=900)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)



    mask = cv2.inRange(hsv, pink_lower, pink_upper)

    mask = cv2.erode(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,

                            cv2.CHAIN_APPROX_SIMPLE)[-2]

    center = None



    if len(cnts) > 0:

        c = max(cnts, key=cv2.contourArea)

        ((x, y), radius) = cv2.minEnclosingCircle(c)

        M = cv2.moments(c)

        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))



        if radius > 10:

            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            a = int(x)

            b = int(y)

            r = int(radius)

            print "x=", a, "y=", b, "z=", r

            key = cv2.waitKey(1) & 0xFF



            if x > 0 and x < 300:

                forward(motor1, 30, motor2, 0)

            elif x > 300 and x < 600:

                forward(motor1, 70, motor2, 70)

            elif x > 600 and x < 900:

                forward(motor1, 0, motor2, 30)

            else:

                forward(motor1, 70, motor2, 70)



        if radius > 200:

            forward(motor1, 0, motor2, 0)

            # Position(servo,15)

            wp.delay(2000)

            Sweep(servo, 100)

            break



    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):

        forward(motor1, 0, motor2, 0)

        break



camera.release()

cv2.destroyAllWindows()

