#coding: UTF-8
import sys

sys.path.append('../Rcb4Lib') #Rcb4Libの検索パスを追加
from Rcb4BaseLib import Rcb4BaseLib #Rcb4BaseLib.pyの中のRcb4BaseLibが使えるように設定

import time 
import cv2
from cv2 import aruco
import numpy as np
import math

rcb4 = Rcb4BaseLib()
rcb4.open('/dev/ttyUSB0',115200,1.3)

cap = cv2.VideoCapture(0)

global tarx
global tary

tarx = 0
tary = 0

def degpos(ang):
    if ang <= 2000:
        ang = 2000
    if ang >= 15000:
        ang = 15000
    deg = ang
    deg = -(deg - 9000)
    pos = (deg* 2963) / 10000
    pos = pos + 7500
    return pos

def calcAngleFunc(anglePlate):
    lengthA = 21
    lengthB = 35
    lengthR = 93
    thetaPlate = (-anglePlate) * (math.pi / 180)
    posA = [lengthR * math.cos(thetaPlate), lengthR * math.sin(thetaPlate)]
    posB = [lengthR, -math.sqrt(lengthB * lengthB - lengthA * lengthA)]

    lengthC = math.sqrt((posB[0] - posA[0])**2 + (posB[1] - posA[1])**2)
    
    if(lengthB - lengthA < lengthC and lengthC <lengthB + lengthA):
        cosBvalue = (lengthA * lengthA + lengthC * lengthC - lengthB * lengthB) / (2 * lengthA * lengthC)
        thetaB = math.acos(cosBvalue)
        thetaH = math.atan2(posA[1] - posB[1], posA[0] - posB[0])       
        servoAngle = 270 - (thetaH + thetaB) * (180 / math.pi)       
        return servoAngle*100 
    else:
        return 9000

def getMask(l, u):
    hsv = np.array([[[255,0,0]]*100]*100)
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    lower = np.array(l)
    upper = np.array(u)
    
    if lower[0] >= 0:
        mask = cv2.inRange(hsv, lower, upper)
    else:
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        mask = np.zeros(h.shape, dtype=np.uint8)
        mask[((h < lower[0]*-1) | (h > upper[0])) & (s > lower[1])] = 255
        
    return cv2.bitwise_and(frame,frame, mask= mask)


def getContours(img,t,r):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    ret, thresh = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=cv2.contourArea, reverse=True)

    if len(contours) > 0:
        cnt = contours[0]
        (x,y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)

        if radius > r:
            return cv2.circle(frame,center,radius,(0,255,0),2),center
        else:
            center = (320,240)
            return cv2.circle(frame,center,5,(0,0,255),2),center
    else:
        center = (320,240)
        return cv2.circle(frame,center,5,(0,0,255),2),center

if rcb4.checkAcknowledge() == True:
    print ('checkAcknowledge OK')
    print ('Version    -->' ,rcb4.Version)

    posa = degpos(9000)
    posb = degpos(9000)
    rcb4.setSingleServo(0,1,int(posa),20)
    rcb4.setSingleServo(1,1,int(posb),20)
else:
    print ('checkAcknowledge error')

diffx = [0,0]
diffy = [0,0]
integralx = 0;
integraly = 0;
normal_prex= 0;
normal_prey= 0;
t= 200
dx = 0
dy = 0
xt= 100
yt= 100

try:
    while True:
        ret, frame = cap.read()
        Height, Width = frame.shape[:2] 
        img = cv2.resize(frame,(int(Width),int(Height)))

        res_red = getMask([95,50,50], [110,255,255])
        contours_frame = getContours(res_red, 50, 25)
        ar_data = 1
        center = contours_frame[1]

        if ar_data == 1:
            normal_x = float((center[0]-320) + normal_prex)/2
            normal_prex = float(center[0]-320)
            # -240 to 240
            normal_y = -(float(center[1]-240) + normal_prey)/2
            normal_prey = float(center[1]-240)
            # -160 to 160
            ar_normal = [normal_x,normal_y]
            
            if(xt > -100):
                xt -= 1
            else:
                if(yt > -100):
                    yt -= 1
            
            target = [0, 0]
            
            kp = 0.032
            ki = 1.227
            kd = 0.01
            delta = 0.033
            diffx[0] = diffx[1]
            diffx[1] =(target[0] - normal_x)
            integralx += (diffx[1] + diffx[0]) /2.0 * delta
            
            diffy[0] = diffy[1]
            diffy[1] =(target[1] - normal_y)
            integraly += (diffy[1] + diffy[0]) /2.0 * delta

            px = kp * diffx[1]
            ix = ki * integralx
            dx = kd * (diffx[1] - diffx[0]) / delta
            posx = px + ix + dx
            
            py = kp * diffy[1]
            iy = ki * integraly
            dy = kd * (diffy[1] - diffy[0]) / delta
            posy = py + iy + dy
            
            if((diffy[1] + diffy[0]) > 50):
                if(dy != 1):
                    integraly = 0
                dy = 1
            elif((diffy[1] + diffy[0]) < -50):
                if(dy != 0):
                    integraly = 0
                dy = 0
            if((diffx[1] + diffx[0]) > 50):
                if(dx != 1):
                    integralx = 0
                dy = 1
            elif((diffx[1] + diffx[0]) < -50):
                if(dx != 0):
                    integralx = 0
                dx = 0
            
            if(posx > 0):
                posx = posx * 1.15
            if(posx < -8.6):
                posx = -8.6
            if(posx > 8.6):
                posx = 8.6
            
            if(posy > 0):
                posy = posy * 1.15
            if(posy < -8.6):
                posy = -8.6
            if(posy > 8.6):
                posy = 8.6
            
            frx = 1
            fry = 1
            if(abs(diffx[1]) < 15):
                integralx = 0
                posx = 0
            if(abs(diffy[1]) < 15):
                integraly = 0
                posy = 0

            if(abs(diffx[1] - diffx[0]) < 5):
                frx = 3
            if(abs(diffy[1] - diffy[0]) < 5):
                fry = 3
            
            if(abs(posx) > 6 and abs(posy) > 6):
                if(abs(posx) > abs(posy)):
                    posx = posx /2
                else:
                    posy = posy /2
                   
            
            rcb4.setSingleServo(0,1,int(degpos(int(calcAngleFunc(posx)))),frx)
            rcb4.setSingleServo(1,1,int(degpos(int(calcAngleFunc(posy)))),fry)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyWindow('frame')
    cap.release()
except KeyboardInterrupt:
    cv2.destroyWindow('frame')
    cap.release()

rcb4.close()
