#!/usr/bin/env python
import serial
import xbox
import pigpio 
import time
import threading
import pyudev
import rospy
context = pyudev.Context()
pi = pigpio.pi()


variable = 0

################### Connecting to Tri-Track ###################

pwmPin1 = 19 #Right side/CH1    
pwmPin2 = 12 #Left side/CH2

# Duty Cycles
dutyCycleForward = 18
dutyCycleBack = 14
dutyCycleStop = 16
dutyCycleTurn = [dutyCycleForward, dutyCycleBack]

# Default values stop motors
dutyCycleMotor1 = dutyCycleStop 
dutyCycleMotor2 = dutyCycleStop

# PWM frequency
frequency = 100

rospy.init_node('listener', anonymous=True)


def connectTriTrack():
    while variable == 0:
        try:
            pi.hardware_PWM(pwmPin1, frequency, int(dutyCycleMotor1*10000))
            pi.hardware_PWM(pwmPin2, frequency, int(dutyCycleMotor2*10000))
            time.sleep(0.01)

        except:
            print('Retrying PWM...')
            time.sleep(2)

connectTriTrackThread = threading.Thread(target=connectTriTrack, args=())
connectTriTrackThread.start()


################### Connecting to the BotBoard/Arm ###################

def connectArduino():
    global arduino
    while variable == 0:
        try:
            for device in context.list_devices(subsystem='tty', ID_SERIAL_SHORT='A49BK0V'):
                port = '/dev/' + device.sys_name
            arduino = serial.Serial(port, baudrate = 115200, timeout = 1)
            print('Connected to Arduino')
            sendData(arduino)
            return

        except:
            print('Searching for Arduino...')
            time.sleep(2)

def sendData(ser):
    while variable == 0:
        ser.write((servoAngleMessage(baseAngle, shoulderAngle, elbowAngle, wristAngle, wristRotateAngle, gripperAngle)).encode())
        
connectArduinoThread = threading.Thread(target=connectArduino, args=())
connectArduinoThread.start()



################### Connecting to the Controller/changing angles ###################

def servoAngleMessage(baseAngle, shoulderAngle, elbowAngle, wristAngle, wristRotateAngle, gripperAngle):
    angleArray = [baseAngle,shoulderAngle,elbowAngle,wristAngle,wristRotateAngle,gripperAngle]
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

    message = message + '\n' #adds newline character to message
    #print(message)

    return(message)


#Servo Angles starting positions (non zero values)
baseAngle = 75
shoulderAngle = 135
elbowAngle = 135
wristAngle = 180
wristRotateAngle = 120
gripperAngle = 1 #These are all the default values

gripperBool = False
 
controllerState = False

ARM_FORWARD_DELAY = 0.02
ARM_REVERSE_DELAY = 0.02
ARM_TURN_DELAY = 0.02


while not rospy.is_shutdown():
    try:
        joy = xbox.Joystick()

        if controllerState == False:
            print('\nConnected to controller')
            controllerState = True
        
        dutyCycleMotor1 = dutyCycleStop  #left motor
        dutyCycleMotor2 = dutyCycleStop  #right motor
        
        ############# D-pad ##############
        
        while joy.dpadUp() == 1:
            #print('DPAD Up')
            time.sleep(0.01)
            if wristAngle >= 180:
                pass
            else:
                wristAngle = wristAngle+1
            
        while joy.dpadDown() == 1:
            #print('DPAD Down')
            time.sleep(0.01)
            if wristAngle <= 1:
                pass
            else:
                wristAngle = wristAngle-1

        while joy.dpadLeft() == 1:     
            print('DPAD Left')

        while joy.dpadRight() == 1:     
            print('DPAD Right')


        ########### Face buttons ###########
            
        while joy.A() == 1:
            print('A')
            if gripperBool == False:
                gripperAngle = 180 #180 = tightly shut
                gripperBool = True
            else:
                gripperAngle = 1
                gripperBool = False
            time.sleep(1)
                
        while joy.B() == 1:
            print('B')
            baseAngle = 75
            shoulderAngle = 130
            elbowAngle = 130
            wristAngle = 180
            wristRotateAngle = 115
                
            time.sleep(1)
            
        while joy.X() == 1:
            print('X')
        while joy.Y() == 1:
            print('Y')


        ######### Left analog stick ##########
            
        #while abs(joy.leftX())>0.15 or abs(joy.leftY())>0.15: # Prevent this loop from being entered incorrectly due to analog sticks sticking at very low values above zero
        #    print('Left x ', joy.leftX())
        #    print('Left y ', joy.leftY())


        ########## Right analog stick ###########

        while abs(joy.rightX())>0.15 or abs(joy.rightY())>0.15: # Prevent this loop from being entered incorrectly due to analog sticks sticking at very low values above zero
            #print('Right x ', joy.rightX())
            #print('Right y ', joy.rightY())
            #time.sleep(0.02)

            #Top quadrant of xy plane - stick pushed up
            if joy.rightY()>=0:

                #Shoulder and Elbow forward
                if joy.rightY()>0.15:
                    time.sleep(ARM_FORWARD_DELAY)
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
                if joy.rightX()>0.15:
                    time.sleep(ARM_TURN_DELAY)
                    if baseAngle >= 180:
                        pass
                    else:
                        baseAngle = baseAngle+1
                elif joy.rightX()<(-0.15):
                    time.sleep(ARM_TURN_DELAY)
                    if baseAngle <= 1:
                        pass
                    else:
                        baseAngle = baseAngle-1


            #Bottom quadrant of xy plane - stick pushed down
            elif joy.rightY()<=0:

                #Shoulder and Elbow back
                if joy.rightY()<(-0.15):
                    time.sleep(ARM_REVERSE_DELAY)
                    if elbowAngle == shoulderAngle:
                        if shoulderAngle >= 170:
                            pass
                        else:
                            shoulderAngle = shoulderAngle+1
                            elbowAngle = elbowAngle+1
                    else:
                        elbowAngle = elbowAngle-1


                #Moving arm left and right
                if joy.rightX()>0.15:
                    time.sleep(ARM_TURN_DELAY)
                    if baseAngle >= 180:
                        pass
                    else:
                        baseAngle = baseAngle+1
                elif joy.rightX()<(-0.15):
                    time.sleep(ARM_TURN_DELAY)
                    if baseAngle <= 1:
                        pass
                    else:
                        baseAngle = baseAngle-1

        ########### Right trigger ###########
                    
        while joy.rightTrigger()>0:
            #print('Right Trigger ', joy.rightTrigger())
            time.sleep(0.01)

            val = joy.rightTrigger()

            #dutyCycleMotor1 = 16 + ((val**2) * 2)
            #dutyCycleMotor2 = 16 + ((val**2) * 2)

            
            if abs(joy.leftX())>0.15 or abs(joy.leftY())>0.15: #if the left trigger has been engaged

                valStick = abs(joy.leftX()) #between 0 and 1

                if joy.leftX()<(-0.15): #pushed to the left
                    dutyCycleMotor1 = 16 + ((val**2) * 2)
                    dutyCycleMotor2 = 16 + ( ((val**2) * 2) * (1-(valStick**2)) )

                elif joy.leftX()>(0.15): #pushed to the right
                    dutyCycleMotor1 = 16 + ( ((val**2) * 2) * (1-(valStick**2)) )
                    dutyCycleMotor2 = 16 + ((val**2) * 2)

            else:
                dutyCycleMotor1 = 16 + ((val**2) * 2)
                dutyCycleMotor2 = 16 + ((val**2) * 2)
            

        ############ Left trigger ############
            
        while joy.leftTrigger()>0:
            time.sleep(0.01)
            #print('Left Trigger ', joy.leftTrigger())
            time.sleep(0.01)

            val = joy.leftTrigger()
            
            #dutyCycleMotor1 = 16 - ((val**2) * 2)
            #dutyCycleMotor2 = 16 - ((val**2) * 2)

            if abs(joy.leftX())>0.15 or abs(joy.leftY())>0.15: #if the left trigger has been engaged

                valStick = abs(joy.leftX()) #between 0 and 1

                if joy.leftX()<(-0.15): #pushed to the left
                    dutyCycleMotor1 = 16 - ((val**2) * 2)
                    dutyCycleMotor2 = 16 - ( ((val**2) * 2) * (1-(valStick**2)) )

                elif joy.leftX()>(0.15): #pushed to the right
                    dutyCycleMotor1 = 16 - ( ((val**2) * 2) * (1-(valStick**2)) )
                    dutyCycleMotor2 = 16 - ((val**2) * 2)

            else:
                dutyCycleMotor1 = 16 - ((val**2) * 2)
                dutyCycleMotor2 = 16 - ((val**2) * 2)


        ######### Right shoudler button ##########
            
        while joy.rightBumper() == 1:
            #print('Right Bumper ', joy.rightBumper())
            time.sleep(0.01)
            if wristRotateAngle >= 180:
                pass
            else:
                wristRotateAngle = wristRotateAngle+1
                
                      
        ########## Left shoulder button ##########
                
        while joy.leftBumper() == 1:
            #print('Left Bumper ', joy.leftBumper())
            time.sleep(0.01)
            if wristRotateAngle <= 1:
                pass
            else:
                wristRotateAngle = wristRotateAngle-1

        joy.close()

    except:
        controllerState = False
        print('Searching for controller...')
        time.sleep(0.5)


def myhook():
    global variable
    variable = 1

  
rospy.on_shutdown(myhook)
