import RPi.GPIO as gpio
from time import sleep
from time import time
import math
import matplotlib.pyplot as plt
import numpy as np

Direction = True
FORWARD = True
BACKWARD = False

spd_prevTime = time()
pulseCnt = 0
pid_prevTime = time()
prevErr = 0
accErr = 0

desiredRPS = 0

hall = 18
hallB = 23
pinCtrlFwdLeft = 19
pinCtrlSpdLeft = 26
pinCtrlFwdRight = 20
pinCtrlSpdRight = 21

gpio.setmode(gpio.BCM)
gpio.setwarnings(False)

gpio.setup(hall, gpio.IN)  #  A
gpio.setup(hallB, gpio.IN)  #  B, must not "pull_up_down = gpio.PUD_UP" !!!!
gpio.setup(pinCtrlFwdLeft, gpio.OUT)
gpio.setup(pinCtrlSpdLeft, gpio.OUT)
myPWMLeft = gpio.PWM(pinCtrlSpdLeft, 2000) # pin, frequency
myPWMLeft.start(0) # 0 ~ 100. So, 50 is same as analogWrite(11, 128)
gpio.setup(pinCtrlFwdRight, gpio.OUT)
gpio.setup(pinCtrlSpdRight, gpio.OUT)
myPWMRight = gpio.PWM(pinCtrlSpdRight, 2000) # pin, frequency
myPWMRight.start(0) # 0 ~ 100. So, 50 is same as analogWrite(11, 128)

pinAPrevState = False

def cntHallSensorPulse(channel):
    global pulseCnt, pinAPrevState, Direction
    Astate = gpio.input(hall)
    Bstate = gpio.input(hallB)
    #print("A: "+str(Astate))
    if (pinAPrevState == gpio.LOW) and Astate == gpio.HIGH:
        #print("FUCK3")
        #print("B: "+str(Bstate))
        if Bstate == gpio.LOW and Direction:
            #print("FUCK")
            Direction = BACKWARD
        elif Bstate == gpio.HIGH and (not Direction):
            Direction = FORWARD
            #print("FUCK2")
    pinAPrevState = Astate
    if (not Direction):
        pulseCnt+=1
    else:
        pulseCnt-=1
    #print(pulseCnt)


def getMotorSpd():
    global spd_prevTime, pulseCnt
    nowTime = time()
    elapsedTime = nowTime - spd_prevTime
    #print(elapsedTime)
    rps = pulseCnt * (1/elapsedTime) /260.0
    spd_prevTime = nowTime
    pulseCnt = 0
    return rps

gpio.add_event_detect(hallB, gpio.BOTH, callback=cntHallSensorPulse, bouncetime=1)
#gpio.add_event_callback(19, cntHallSensorPulse)

def setMotorSpd(spd):
    if spd >= 0:
        gpio.output(pinCtrlFwd, gpio.HIGH)
        spd = 250 if spd > 250 else spd
    else:
        gpio.output(pinCtrlFwd, gpio.LOW)
        spd = -250 if spd < -250 else spd
        spd = -1 * spd
    print("spd: " + str(spd))
    convertPWM = (float(spd)/255*100)
    myPWM.ChangeDutyCycle((int)(convertPWM))

def computePID(rps):
    global desiredRPS, pid_prevTime, prevErr, accErr
    ## Tune PID controller parameters !
    kp = 50.0
    ki = 0.01
    kd = 0.0

    nowTime = time()
    elapsedTime = (nowTime - pid_prevTime) * 1000 # convert to milli

    Err = desiredRPS - rps
    accErr += Err * elapsedTime
    rateErr = (Err - prevErr) / elapsedTime

    output = kp*Err + ki*accErr + kd*rateErr + Err

    pid_prevTime = nowTime
    prevErr = Err

    return output


def mvStraight(spd):
    if spd > 0 :
        gpio.output(pinCtrlFwdLeft, gpio.LOW)
        gpio.output(pinCtrlFwdRight, gpio.HIGH)
    else:
        gpio.output(pinCtrlFwdLeft, gpio.HIGH)
        gpio.output(pinCtrlFwdRight, gpio.LOW)
    myPWMLeft.ChangeDutyCycle((abs)((int)(spd)))
    myPWMRight.ChangeDutyCycle((abs)((int)(spd)))

def mvRotate(spd, steer):
    steer = -1 if steer < -1 else steer
    steer = 1 if steer > 1 else steer
    # steer > 0, right
    if steer > 0:
        spdLeft = spd
        spdRight = spd * (1 - steer)
    if steer <= 0:
        spdLeft = spd * (1- abs(steer))
        spdRight = spd
    if spd > 0 :
        gpio.output(pinCtrlFwdLeft, gpio.LOW)
        gpio.output(pinCtrlFwdRight, gpio.HIGH)
    else:
        gpio.output(pinCtrlFwdLeft, gpio.HIGH)
        gpio.output(pinCtrlFwdRight, gpio.LOW)
    spdLeft = 0 if spdLeft < 0 else spdLeft
    spdLeft = 100 if spdLeft > 100 else spdLeft
    spdRight = 0 if spdRight < 0 else spdRight
    spdRight = 100 if spdRight > 100 else spdRight
    myPWMLeft.ChangeDutyCycle((int)(spdLeft))
    myPWMRight.ChangeDutyCycle((int)(spdRight))
    #print("Left : " + str(spdLeft))
    #print("Right: " + str(spdRight))
    #print(" ")

def mvPivot(spd, steer):
    if steer > 0 :
        gpio.output(pinCtrlFwdLeft, gpio.LOW)
        gpio.output(pinCtrlFwdRight, gpio.LOW)
    else:
        gpio.output(pinCtrlFwdLeft, gpio.HIGH)
        gpio.output(pinCtrlFwdRight, gpio.HIGH)
    myPWMLeft.ChangeDutyCycle((int)(spd))
    myPWMRight.ChangeDutyCycle((int)(spd))

if __name__ == '__main__':
    import cv2
    verbose = False

    try:
        startTime = time()
        desiredRPS = -3.0
        '''
    while True:

        x = time() - startTime
        rps = getMotorSpd()
        if verbose:
            plt.scatter(x,rps)
            plt.pause(0.001)
        print("rps: " + str(rps))
        output = computePID(rps)
        setMotorSpd(output)
        print()
        sleep(0.1)
        '''
        spd = 50
        temp = np.zeros((100, 100, 1), dtype=np.uint8)
        while True:
            cv2.imshow("asdf", temp)
            key = cv2.waitKey(1)
            #print(key)
            if key == ord('w'):
                print("FUCK")
                mvStraight(spd)
            if key == ord('s'):
                mvStraight(-spd)
            if key == ord('a'):
                mvRotate(spd, -1)
            if key == ord('d'):
                mvRotate(spd, 1)
            if key == ord('q'):
                mvRotate(0,0)

            # set spd 0 ~ 100
    # Quit on Ctrl-c
    except KeyboardInterrupt:
        print "Ctrl-C - quit"


    # Cleanup GPIO
    finally:
        gpio.cleanup()
        plt.close()
        print("GPIO CLENAUP FINISH")
