import cv2
import numpy as np
import serial
import math

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

W, H = (640, 480)

cap.set(3, W)
cap.set(4, H)

diag = math.sqrt(W**2 + H**2)

_, frame = cap.read()
rows, cols, _ = frame.shape

x_medium = int(cols / 2)
center = int(cols / 2)
position = 90 # degrees
loopdelay = 0
loop = 0
size = 0
size_treshold = 5

red_ball_size = 105 #[mm]
frame_size_at_250mm = 50 #[percent od screen diag]
distance_0 = 260 # [mm]
distance = 0

dist_to_size = distance_0 * frame_size_at_250mm

Kp_angle = 0.1
Kp_dist = 0.025 

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # red color
    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    
    # for cnt in contours:
    #     (x, y, w, h) = cv2.boundingRect(cnt)

    #     # calculating the size
    #     size = 100 * math.sqrt(w**2 + h**2) / diag
    #     x_medium = int((x + x + w) / 2)
    #     break
    
    if len(contours) > 0:
        cnt = contours[0]
        (x, y, w, h) = cv2.boundingRect(cnt)

        # calculating the size
        size = 100 * math.sqrt(w**2 + h**2) / diag
        distance = int(dist_to_size / size) 

        x_medium = int(x + w / 2)
        y_medium = int(y + h / 2)
    
    
    if size >= size_treshold:
        cv2.line(frame, (x_medium, 0), (x_medium, H), (0, 255, 0), 2)
        cv2.line(frame, (0, y_medium), (W, y_medium), (0, 255, 0), 2)
        
        # moving around
        turn_error = center - x_medium;
        dist_error = distance - 300;

        turn_angle = 0;
        move_distance = 0;

        
        if abs(turn_error) > 30:
            turn_angle = int(Kp_angle * turn_error)            

        if abs(dist_error) > 100:
            move_distance = int(Kp_dist * dist_error)

        # if move_distance != 0 or turn_angle != 0:
        if True: 
            message = f'<1,{move_distance}, {turn_angle} ,500>'
            print(message.encode('utf-8'))
            
            if loop > loopdelay:    
                loop = 0
                try:
                    ser.write(message.encode('utf-8'))
                except:
                    pass
        else:
            if True: 
                message = f'<8,0,0,0>'
                print(message.encode('utf-8'))
            
                if loop > loopdelay:    
                    loop = 0
                    try:
                        ser.write(message.encode('utf-8'))
                    except:
                        pass

    
    cv2.putText(frame,str(int(size)), (10,10), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
    cv2.putText(frame,str(distance), (10,25), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
    cv2.imshow("Frame", frame)
    
    loop += 1;

    key = cv2.waitKey(1)
    
    if key == 27:
        break
    
cap.release()
cv2.destroyAllWindows()
