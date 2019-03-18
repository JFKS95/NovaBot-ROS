#!/usr/bin/env python
import xbox
import time
import threading

#for keyboard control
import pygame, sys
from pygame.locals import *


################### Connecting to the Controller/changing angles ###################

#Servo Angles starting positions (non zero values)
baseAngle = 80
shoulderAngle = 170
elbowAngle = 170
wristAngle = 90
wristRotateAngle = 30
gripperAngle = 1 #These are all the default values

#Tri track states
forward = 0
reverse = 0
left = 0
right = 0


def getServoArmMessage(baseA, shoulderA, elbowA, wristA, wristRotateA, gripperA):
    angleArray = [baseA,shoulderA,elbowA,wristA,wristRotateA,gripperA]
    strArray = []

    for x in range(0,6):
        angleLen = len(str(angleArray[x]))
        
        if angleLen == 1:
            newAngle = '00' + str(angleArray[x])
            strArray.append(newAngle)

        elif angleLen == 2:
            newAngle = '0' + str(angleArray[x])
            strArray.append(newAngle)

        else:
            newAngle = str(angleArray[x])
            strArray.append(newAngle)

    message = ''

    for x in range(0,6):
        message = message + strArray[x] #adds letters a,b,c,d,e,f for identifying the servos

    #message = message + '\n' #adds newline character to message
    
    return(message)


def getTriTrackMessage():
    servoArmMessage = getServoArmMessage(baseAngle, shoulderAngle, elbowAngle, wristAngle, wristRotateAngle, gripperAngle)
    message = servoArmMessage + str(forward) + str(reverse) + str(left) + str(right)
    return(message)


def xboxController():
    global baseAngle 
    global shoulderAngle
    global elbowAngle
    global wristAngle
    global wristRotateAngle 
    global gripperAngle

    global forward
    global reverse
    global left
    global right

    gripperBool = False

    while True:
        try:
            joy = xbox.Joystick()
            print('Connected to controller')
            
            while not joy.Back():
                forward = 0
                reverse = 0
                left = 0
                right = 0
                
                #D-pad
                while joy.dpadUp() == 1:     
                    #print('DPAD Up')
                    forward = 1

                while joy.dpadDown() == 1:
                    #print('DPAD Down')
                    reverse = 1

                while joy.dpadLeft() == 1:     
                    #print('DPAD Left')
                    left = 1

                while joy.dpadRight() == 1:     
                    #print('DPAD Right')
                    right = 1

                #Face buttons
                while joy.A() == 1:
                    #print('A')
                    if gripperBool == False:
                        gripperAngle = 180 #180 = tightly shut
                        gripperBool = True
                    else:
                        gripperAngle = 1
                        gripperBool = False
                    time.sleep(1) #long delay to prevent state changing too quickly and not allowing gripper to fully open/shut
                        
                while joy.B() == 1:
                    print('B')
                while joy.X() == 1:
                    print('X')
                while joy.Y() == 1:
                    print('Y')

                #Left analog stick
                while abs(joy.leftX())>0 or abs(joy.leftY())>0:
                    #print('Left x ', joy.leftX())
                    #print('Left y ', joy.leftY())
                    time.sleep(0.01)

                    #Top quadrant of xy plane
                    if joy.leftY()>=0:

                        #Shoulder and Elbow forward
                        if joy.leftY()>0.15:
                            if shoulderAngle <= 20:
                                if elbowAngle >50:
                                    pass
                                else:
                                    elbowAngle = elbowAngle+1
                                pass
                            else:
                                shoulderAngle = shoulderAngle-1
                                elbowAngle = elbowAngle-1

                        #Moving arm left and right 
                        if joy.leftX()>0.15:
                            if baseAngle >= 180:
                                pass
                            else:
                                baseAngle = baseAngle+1
                        elif joy.leftX()<(-0.15):
                            if baseAngle <= 1:
                                pass
                            else:
                                baseAngle = baseAngle-1


                    #Bottom quadrant of xy plane       
                    elif joy.leftY()<=0:

                        #Shoulder and Elbow back
                        if joy.leftY()<(-0.15):
                            if elbowAngle == shoulderAngle:
                                if shoulderAngle >= 170:
                                    pass
                                else:
                                    shoulderAngle = shoulderAngle+1
                                    elbowAngle = elbowAngle+1
                            else:
                                elbowAngle = elbowAngle-1


                        #Moving arm left and right
                        if joy.leftX()>0.15:
                            if baseAngle >= 180:
                                pass
                            else:
                                baseAngle = baseAngle+1
                        elif joy.leftX()<(-0.15):
                            if baseAngle <= 1:
                                pass
                            else:
                                baseAngle = baseAngle-1




                #Right analog stick
                while abs(joy.rightX())>0.15 or abs(joy.rightY())>0.15:
                    #print('Right x ', joy.rightX())
                    #print('Right y ', joy.rightY())
                    time.sleep(0.005)

                    if joy.rightY()>0.15:
                        if wristAngle >= 180:
                            pass
                        else:
                            wristAngle = wristAngle+1
                    elif joy.rightY()<(-0.15):
                        if wristAngle <= 1:
                            pass
                        else:
                            wristAngle = wristAngle-1

                #Right trigger
                while joy.rightTrigger()>0:
                    #print('Right Trigger ', joy.rightTrigger())
                    time.sleep(0.01)
                    if wristRotateAngle >= 180:
                        pass
                    else:
                        wristRotateAngle = wristRotateAngle+1

                #Left trigger
                while joy.leftTrigger()>0:
                    #print('Left Trigger ', joy.leftTrigger())
                    time.sleep(0.01)
                    if wristRotateAngle <= 1:
                        pass
                    else:
                        wristRotateAngle = wristRotateAngle-1


                #Right shoudler button
                while joy.rightBumper() == 1:
                    #print('Right Bumper ', joy.rightBumper())
                    if baseAngle == 180:
                        pass
                    else:
                        baseAngle = baseAngle+1
                              
                #Left shoulder button
                while joy.leftBumper() == 1:
                    #print('Left Bumper ', joy.leftBumper())
                    if baseAngle == 1:
                        pass
                    else:
                        baseAngle = baseAngle-1

            joy.close()
         
        except:
            print('Searching for controller...')
            time.sleep(0.5)

xboxThread = threading.Thread(target=xboxController, args=())
xboxThread.start()

################### Keyboard control ########################


def keyboardControl():
    global forward
    global reverse
    global left
    global right 
    
    pygame.init()

    COLOUR = (0,0,0)
    WIDTH = 640
    HEIGHT = 480
    windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

    windowSurface.fill(COLOUR)
    
    while True:
        forward = 0
        reverse = 0
        left = 0
        right = 0
        
        #get all the user events
        for event in pygame.event.get():

            #if user wants to quit
            if event.type == pygame.locals.QUIT:
                #and the game close the window
                pygame.quit()
                sys.exit()

            #if a key is pressed
            elif event.type == pygame.locals.KEYDOWN:
                if event.key == K_UP:
                    print('up')
                    forward = 1
                if event.key == K_DOWN:
                    reverse = 1
                if event.key == K_LEFT:
                    left = 1
                if event.key == K_RIGHT:
                    right = 1

##keyboardThread = threading.Thread(target=keyboardControl, args=())
##keyboardThread.start()       
