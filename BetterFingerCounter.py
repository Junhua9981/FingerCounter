import cv2
import time
import os
from math import *
import numpy as np
import numba as nb
import HandTrackingModule as htm

__author__="Junhua"
__updateDate__="2021/10/26"


def rtList(lmList:list, angle:float):
    rotList=[]
    for lm in lmList:
        new_v=rotate( (lm[1]-lmList[0][1],lm[2]-lmList[0][2]) , angle)
        rotList.append((new_v[0]+lmList[0][1],new_v[1]+lmList[0][2]))
    return rotList


def calcAngel(vect2:list):
    v1 = np.asarray((0,1))
    v2 = np.asarray(vect2)
    # print (v2)
    # 計算方向向量
    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)
    # 根據dot反推夾角
    dot_product = np.dot(unit_v1, unit_v2)
    angle = np.arccos(dot_product)
    cross_product = np.cross(unit_v1,unit_v2)
    if cross_product.item() > 0:
        angle = 2*pi - angle
    # print(angle*180/pi)
    return angle

def rotate(vect2: list, angle: float):
    v2 = np.asarray(vect2)
    rotate_matrix = np.asarray([[cos(angle),-sin(angle)],[sin(angle),cos(angle)]])
    new_v2 = np.dot(rotate_matrix,v2) 
    return (int(new_v2[0]),int(new_v2[1]))
# via:https://blog.csdn.net/u012570215/article/details/108226029
# bug fixed by me



def fingerCounter():
    wCam, hCam = 640, 480
    # wCam, hCam =1280,720
    cap = cv2.VideoCapture(0)
    cap.set(3, wCam)
    cap.set(4, hCam)

    pTime = 0

    detector = htm.handDetector(detectionCon=0.75)

    tipIds = [4, 8, 12, 16, 20]
    rotTipIds = [3, 7, 11, 15, 19]

    prevHand=0
    prevCounter=0
    rotList=[]
    # imgCanvas = np.zeros((720, 1280, 3), np.uint8)


    while True:
        success, img = cap.read()
        img = detector.findHands(img)
        imgCanvas = np.zeros((hCam, wCam, 3), np.uint8)
        lmList = detector.findPosition(img, draw=False)
        # print(lmList)
        rotList=[]
        if len(lmList) != 0:

            angle=calcAngel((lmList[0][1]-lmList[9][1],lmList[0][2]-lmList[9][2]))
            rotList=rtList(lmList[1:],angle)

            fingers = []
            flip=False

            if rotList[rotTipIds[0]][0]>rotList[rotTipIds[4]][0]:
                flip=True
            else :
                flip =False

            # Thumb
            if flip:
                if rotList[rotTipIds[0]][0] > rotList[rotTipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            else :
                if rotList[rotTipIds[0]][0] < rotList[rotTipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)

            # display 拇指座標
            # cv2.putText(img, str(lmList[tipIds[0]][1:]), lmList[tipIds[0]][1:],cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 255), 2)
            
            # 4 Fingers
            for id in range(1, 5):
                if rotList[rotTipIds[id]][1] < rotList[rotTipIds[id] - 2][1]:
                    fingers.append(1)
                else:
                    fingers.append(0)
                # display 4 fingersㄉ座標
                # cv2.putText(img, str(lmList[tipIds[id]][1:]), lmList[tipIds[id]][1:],cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 255), 2)
            
            # 顯示那些手指頭是打開的 1:opened 0:not opened
            # print(fingers)
            # 特殊案例
            if fingers[0] and fingers[4] and not fingers[1] and not fingers[2] and not fingers[3]:
                totalFingers=6
                # print(6)
            elif fingers[0] and fingers[1] and not fingers[2] and not fingers[3] and not fingers[4]:
                totalFingers=7
                # print(7)
            elif fingers[0] and fingers[1] and fingers[2] and not fingers[3] and not fingers[4]:
                totalFingers=8
                # print(8)
            elif fingers[0] and fingers[1] and fingers[2] and fingers[3] and not fingers[4]:
                totalFingers=9
                # print(9)
            elif not fingers[0] and fingers[1] and not fingers[2] and not fingers[3] and fingers[4]:
                twoDigit=True
                continue
            else:
                totalFingers = fingers.count(1)

            if totalFingers==prevHand and totalFingers != 0:
                prevCounter+=1
            else:
                prevCounter=0
                prevHand=totalFingers
            cv2.rectangle(img, (40, 300), (140, 400), (0, 255, 0), cv2.FILLED)
            if prevCounter>20:
                isJudgeSuccess=True
                if not flip:
                    outputLevel="B"+str(totalFingers)
                else:
                    outputLevel=" "+str(totalFingers)
                cv2.putText(img, str(outputLevel), (45, 375), cv2.FONT_HERSHEY_PLAIN,5, (255, 0, 0), 5)
            else:
                cv2.putText(img, "Judging", (45, 375), cv2.FONT_HERSHEY_PLAIN,5, (255, 0, 0), 5)

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, f'FPS: {int(fps)}', (400, 70), cv2.FONT_HERSHEY_PLAIN,
                    3, (255, 0, 0), 3)

        cv2.imshow("Image", img)

        if len(rotList)!=0:
            cv2.line(imgCanvas, (lmList[0][1],lmList[0][2]), rotList[0], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[0], rotList[1], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[1], rotList[2], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[2], rotList[3], (0, 0, 255), 5)
            cv2.line(imgCanvas, (lmList[0][1],lmList[0][2]), rotList[4], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[4], rotList[5], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[5], rotList[6], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[6], rotList[7], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[4], rotList[8], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[8], rotList[9], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[9], rotList[10], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[10], rotList[11], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[8], rotList[12], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[12], rotList[13], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[13], rotList[14], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[14], rotList[15], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[12], rotList[16], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[16], rotList[17], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[17], rotList[18], (0, 0, 255), 5)
            cv2.line(imgCanvas, rotList[18], rotList[19], (0, 0, 255), 5)
            cv2.line(imgCanvas, (lmList[0][1],lmList[0][2]), rotList[16], (0, 0, 255), 5)

        cv2.imshow("Canvas", imgCanvas)
        if cv2.waitKey(1) & 0xFF == 27:
            break

# https://google.github.io/mediapipe/images/mobile/hand_landmarks.png
if __name__ == "__main__":
    fingerCounter()