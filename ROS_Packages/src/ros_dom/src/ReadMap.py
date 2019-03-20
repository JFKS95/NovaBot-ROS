import cv2
import time

key = cv2.waitKey(1) & 0xFF
while True:
  targetFile = open('TriTrackLoc.txt','r')
  for line in targetFile:
    print(line)
  time.sleep(1)
  targetFile.close()


      
      
