#!/usr/bin/env python3
import serial

xbee = serial.Serial('/dev/ttyUSB0', baudrate=9600)

def getAPITransmitFrame(RFData, destADDR64, destADDR16):
    frameDataLen = 14 + len(RFData) #A full API frame contains 14 fixed bytes + variable data frame bytes
    hexFrameDataLen = frameDataLen.to_bytes(2, 'big') #converting the frame data length to a byte array
    hexDestADDR64 = bytearray.fromhex(destADDR64) #64-bit address converted to byte array
    hexDestADDR16 = bytearray.fromhex(destADDR16) #16-bit address converted to byte array
    hexRFData = bytearray(RFData.encode()) #Message converted to byte array

    APIFrame = []
    APIFrame.extend([0x7e])
    APIFrame.extend(hexFrameDataLen)
    APIFrame.extend([0x10, 0x01])
    APIFrame.extend(hexDestADDR64)
    APIFrame.extend(hexDestADDR16)
    APIFrame.extend([0x00, 0x00])
    APIFrame.extend(hexRFData)

    frameDataSum = 0
    for x in range(3,len(APIFrame)):
        frameDataSum = frameDataSum + APIFrame[x] #adding up all the byte values in the frame data (i.e. all data values between the frame length and checksum)

    hexFrameDataSum = frameDataSum.to_bytes(2, 'big') #converting the above sum to a byte array
    checksum = 0xFF - hexFrameDataSum[1] #calculating the checksum, which uses only the LSB byte of the frame data sum (i.e. the first 8 bits from right side)
    APIFrame.extend([checksum]) #add the checksum to complete the API frame

    return(APIFrame)


destADDR64 = '0013A2004150EBE6' #64-bit address of destination Xbee module (MAC address)
destADDR16 = 'FFFE' #16-bit network address of destination Xbee module (leave as FFFE if unknown)

def sendMessage():
    #RFData = input('Enter the message you want to send: ')
    RFData = 'x'
    APIFrame = getAPITransmitFrame(str(RFData), destADDR64, destADDR16)
    xbee.write(APIFrame)
    print('Message Sent\n')

sendMessage()

'''
(64-bit) mac address of router: 0013A2004150EBE6
(16-bit) address of router: 3985
APIFrame = [0x7e,   #Start delimeter
            0x00, #MSB - Length
            0x1b, #LSB - Length
            0x10,   #Frame data start
            0x01,
            0x00,   #MSB 64-bit address
            0x13,
            0xa2,
            0x00,
            0x41,
            0x50,
            0xeb,
            0xe6,   #LSB 64-bit address
            0x34,   #MSB 16-bit network address
            0xdc,   #LSB 16-bit network address
            0x00,   #Broadcast radius
            0x00,   #Options
            0x48,   #Data to send
            0x65, 
            0x6c, 
            0x6c,
            0x6f,
            0x20,
            0x52,
            0x6f,
            0x75,
            0x74,
            0x65,
            0x72,
            0x21, 
            0x11]   #Checksum
'''
