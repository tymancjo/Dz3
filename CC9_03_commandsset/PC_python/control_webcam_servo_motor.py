import cv2
import numpy as np
import serial

# cams_test = 10
# for i in range(0, cams_test):
#     cap = cv2.VideoCapture(i)
#     test, frame = cap.read()
#     print("i : "+str(i)+" /// result: "+str(test))

#  odpalenie BT com - na linux, z użyciem ble-serial dającego port /tmp/ttyBLE
try:
    ser = serial.Serial('/tmp/ttyBLE', timeout=1)
    ser.baudrate = 9600
except:
    print("Serial BT port error...")

cap = cv2.VideoCapture(2)

cap.set(3, 480)
cap.set(4, 320)

_, frame = cap.read()
rows, cols, _ = frame.shape

x_medium = int(cols / 2)
center = int(cols / 2)
position = 90 # degrees
loopdelay = 0
loop = 0


while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # red color
    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    
    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)
        
        x_medium = int((x + x + w) / 2)
        break
    
    cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
    
    cv2.imshow("Frame", frame)
    
    # moving around
    _error = center - x_medium;

    if abs(_error) > 30:
        message = f'<1,0,{_error/10},300>'

        if loop > loopdelay:    
            loop = 0
            ser.write(message.encode('utf-8'))
            print(message.encode('utf-8'))

    loop += 1;

    key = cv2.waitKey(1)
    
    if key == 27:
        break
    
cap.release()
cv2.destroyAllWindows()
