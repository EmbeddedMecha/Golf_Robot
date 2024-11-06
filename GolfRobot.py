Python 3.12.7 (tags/v3.12.7:0b05ead, Oct  1 2024, 03:06:41) [MSC v.1941 64 bit (AMD64)] on win32
Type "help", "copyright", "credits" or "license()" for more information.
>>> import cv2, sys
... import numpy as np
... import platform
... import serial
... import time
... 
... ser = serial.Serial(
...    port='/dev/ttyS0',
...    baudrate=4800,
...    timeout=0.0001
... )
... ser.flush()
... 
... FORWARD = 11     
... STOP = 26        
... 
... def send_command(command):
...    ser.write(serial.to_bytes([command]))
...    time.sleep(0.1)
... 
... def draw_str(dst, target, s):
...    x, y = target
...    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
...    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)
... 
... def clock():
...    return cv2.getTickCount() / cv2.getTickFrequency()
... 
... W_View_size = 320
... H_View_size = int(W_View_size / 1.333)
... FPS = 90
... 
... cap = cv2.VideoCapture(0)
... cap.set(3, W_View_size)
... cap.set(4, H_View_size)
cap.set(5, FPS)  

lower_pink = np.array([150, 50, 50])
upper_pink = np.array([170, 255, 255])

old_time = clock()
View_select = 1

while True:
   ret, img = cap.read()
   if not ret:
       break
       
   Frame_time = 1000 / ((clock() - old_time) * 1000.)
   old_time = clock()
   
   hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
   mask = cv2.inRange(hsv, lower_pink, upper_pink)
   
   kernel = np.ones((5,5), np.uint8)
   mask = cv2.erode(mask, kernel, iterations=1)
   mask = cv2.dilate(mask, kernel, iterations=1)
   
   contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   
   if len(contours) > 0:
       send_command(FORWARD)
   else:
       send_command(STOP)
   
   if View_select == 1:
       draw_str(img, (5, 20), str(W_View_size) + " x " + str(H_View_size) + ' = FPS %.1f ' % (Frame_time))
       cv2.imshow('Video Test', img)
   
   key = 0xFF & cv2.waitKey(1)
   if key == 27:
       break
   elif key == ord(' '):
       View_select = 1 - View_select

send_command(STOP)
cap.release()
cv2.destroyAllWindows()
ser.close()
