#!/usr/bin/python3
# -*- coding: utf-8 -*-

print('### Script:', __file__)

import math
import sys
import time
import numpy as np
import vrep

SPEED = 1.5
PREFERRED_DISTANCE = 0.25
MINIMUM_COLISION = 0.15
# --------------------------------------------------------------------------

def getRobotHandles(clientID):
    # Motor handles
    _,lmh = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                     vrep.simx_opmode_blocking)
    _,rmh = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                     vrep.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _,h = vrep.simxGetObjectHandle(clientID, str % (i+1),
                                       vrep.simx_opmode_blocking)
        sonar[i] = h
        vrep.simxReadProximitySensor(clientID, h, vrep.simx_opmode_streaming)

    # Camera handles
    _,cam = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_camera',
                                        vrep.simx_opmode_oneshot_wait)
    vrep.simxGetVisionSensorImage(clientID, cam, 0, vrep.simx_opmode_streaming)
    vrep.simxReadVisionSensor(clientID, cam, vrep.simx_opmode_streaming)

    return [lmh, rmh], sonar, cam

# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    vrep.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    vrep.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = vrep.simxReadProximitySensor(clientID, handle,
                                                 vrep.simx_opmode_buffer)
        if e == vrep.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])

    return r

# --------------------------------------------------------------------------

def getImage(clientID, hRobot):
    img = []
    err,r,i = vrep.simxGetVisionSensorImage(clientID, hRobot[2], 0,
                                            vrep.simx_opmode_buffer)

    if err == vrep.simx_return_ok:
        img = np.array(i, dtype=np.uint8)
        img.resize([r[1],r[0],3])
        img = np.flipud(img)
        img = cv.cvtColor(img, cv.COLOR_RGB2BGR)

    return err, img

# --------------------------------------------------------------------------

def getImageBlob(clientID, hRobot):
    rc,ds,pk = vrep.simxReadVisionSensor(clientID, hRobot[2],
                                         vrep.simx_opmode_buffer)
    blobs = 0
    coord = []
    if rc == vrep.simx_return_ok and pk[1][0]:
        blobs = int(pk[1][0])
        offset = int(pk[1][1])
        for i in range(blobs):
            coord.append(pk[1][4+offset*i])
            coord.append(pk[1][5+offset*i])

    return blobs, coord

# --------------------------------------------------------------------------

def getObjectDirection(blobs, coord):
    direction = 0

    if blobs > 0:
        direction = coord[0] * 2 - 1
    return direction

# --------------------------------------------------------------------------

def getObjectDistance(blobs, coord):
    distance = PREFERRED_DISTANCE

    if blobs > 0:
        distance = coord[1]
    return distance

# --------------------------------------------------------------------------

def controlDirPID(lastDirError, errorDirSum, dirError):
    # Constants
    kp = 1.5
    ki = 0.8
    kd = 0.7

    # Algorithm PID
    change = kp * dirError + ki * errorDirSum + kd * (lastDirError - dirError)
    #print("p = ", dirError)
    #print("i = ", errorDirSum)
    #print("d = ", lastDirError - dirError)


    return change

# --------------------------------------------------------------------------

def getSpeed(speed, changeDir, changeDist):
    speed += changeDist
    alpha = 0.8

    #print("PreChangeDir = ", changeDir)
    # Exponential function
    if changeDist >= -SPEED:
        changeDir *= alpha ** (changeDist + SPEED)
    else:
        changeDir *= alpha ** (-changeDist - SPEED)
    #print("PreChangeDir = ", changeDir)

    lspeed = (1 + changeDir) * speed
    rspeed = (1 - changeDir) * speed

    return lspeed, rspeed

# --------------------------------------------------------------------------

def getDistance(sonar):
    distance = min(sonar[2], sonar[3], sonar[4], sonar[5])

    return distance

# --------------------------------------------------------------------------

def controlDistancePID(lastDistanceError, errorDistanceSum, distanceError):
    # Constants
    kp = 2
    ki = 0.05
    kd = 0.2

    # Algorithm PID
    change = kp * distanceError + ki * errorDistanceSum + kd * (lastDistanceError - distanceError)
    #print("p = ", distanceError)
    #print("i = ", errorDistanceSum)
    #print("d = ", (lastDistanceError - distanceError))

    return change

# --------------------------------------------------------------------------

def colisionDetection(sonar, blobs):
    lastMinimum = MINIMUM_COLISION
    flag = False
    lastSonar = 0

    for i in range(16):
        # If the robot is following the ball deactivate the frontal sonar sensors
        if blobs==1 and i in (2, 3, 4, 5):
            continue
        else:
            if sonar[i] <= lastMinimum:
                lastMinimum = sonar[i]
                lastSonar = i
                flag = True

    return flag, lastSonar

# --------------------------------------------------------------------------

def getAvoidDirection(minSonarPosition, speed):

    #    2  3  4  5
    #  1            6
    # 0              7
    #       Sonar
    #15              8
    # 14            9
    #   13 12 11 10

    avoidProportion = 0
    avoidSpeed = 0

    case0_7 = -0.4
    case1_6 = -0.6
    case2_5 = -0.85
    case3_4 = 1.1
    case15_8 = 0.1
    case14_9 = 0.2
    case13_10 = 0.1
    #case11_12 = 0

    print("Sonar ",minSonarPosition)
    if minSonarPosition == 0:
        avoidProportion = case0_7
        avoidSpeed = SPEED * -2
    elif minSonarPosition == 1:
        avoidProportion = case1_6
        avoidSpeed = SPEED * -2
    elif minSonarPosition == 2:
        avoidProportion = case2_5
        avoidSpeed = SPEED * -2
    elif minSonarPosition in (3, 4):
        avoidProportion = case3_4
        avoidSpeed = SPEED * -2
    elif minSonarPosition == 5:
        avoidProportion = - case2_5
        avoidSpeed = SPEED * -2
    elif minSonarPosition == 6:
        avoidProportion = - case1_6
        avoidSpeed = SPEED * -2
    elif minSonarPosition == 7:
        avoidProportion = - case0_7
        avoidSpeed = SPEED * -2
    elif minSonarPosition == 8:
        avoidProportion = - case15_8
    elif minSonarPosition == 9:
        avoidProportion = - case14_9
    elif minSonarPosition == 10:
        avoidProportion = - case13_10
    elif minSonarPosition in (11, 12):
        avoidSpeed *= -2
    elif minSonarPosition == 13:
        avoidProportion = case13_10
    elif minSonarPosition == 14:
        avoidProportion = case14_9
    else: # 15
        avoidProportion = case15_8

    return avoidProportion, avoidSpeed

# --------------------------------------------------------------------------

def main():
    print('### Program started')

    # print '### Number of arguments:', len(sys.argv), 'arguments.'
    # print '### Argument List:', str(sys.argv)

    vrep.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = vrep.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID)

        distancePreferred = 0.4 # TODO cambiar a globales
        lastDir = 0
        lastDirError = 0
        errorDirSum = 0
        lastDistanceError = 0
        errorDistanceSum = 0

        while vrep.simxGetConnectionId(clientID) != -1:

            # Perception
            sonar = getSonar(clientID, hRobot)
            #print('### s', sonar)

            blobs, coord = getImageBlob(clientID, hRobot)
            #print('###  ', blobs, coord)

            # Planning

            flag, minSonarPosition = colisionDetection(sonar, blobs)

            if flag:
                #COLISION CONTROLLER
                avoidProportion, avoidSpeed = getAvoidDirection(minSonarPosition, SPEED)
                lspeed, rspeed = getSpeed(SPEED, avoidProportion, avoidSpeed)
            else:
                #OTHER CONTROLLERS
                direction = getObjectDirection(blobs, coord)
                dirError = direction - lastDir
                errorDirSum += dirError
                changeDir = controlDirPID(lastDirError, errorDirSum, dirError)
                lastDirError = dirError
                lastDir = direction
                print("changeDir = ", changeDir)

                #distance = getObjectDistance(blobs, coord)
                distance = getDistance(sonar)
                if blobs > 0:
                    distanceError = (distance - PREFERRED_DISTANCE)
                else:
                    distanceError = 0
                errorDistanceSum += distanceError
                changeDist = controlDistancePID(lastDistanceError, errorDistanceSum, distanceError)
                #print("Distance = ",distance)
                #print("ChangeDist = ",changeDist)

                lspeed, rspeed = getSpeed(SPEED, changeDir, changeDist)
                #print("l = %f, r = %f" % (lspeed, rspeed))
                #print("Lspeed = %f, Rspeed = %f, direction = %f, change = %f" % (lspeed, rspeed, direction, change))

            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(0.1)

        print('### Finishing...')
        vrep.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
