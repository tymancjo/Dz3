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

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture('rtsp://192.168.0.25/H264?ch=1&subtype=0')

W, H = (1280 // 2, 720 // 2)

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
frame_size_at_250mm = 43 #[percent od screen diag]
distance_0 = 260 # [mm]
distance = 0

# x shift vs distance calibration
dist_c = 634
shift_c = 207
x_shift_cal = dist_c * shift_c
x_shift_to_mm = 250/1.02

last_was_zero = False

dist_to_size = distance_0 * frame_size_at_250mm

Kp_angle = 0.02
Ki_angle = 0.00
Kd_angle = 0.01

angle_death_zone = 1

Kp_dist = 0.1
Ki_dist = 0.0
Kd_dist = 0.01

target_distance = 300 #[mm]
dist_death_zone = 5 #[cm]

# stuff for PID
turn_error_memory = 0;
dist_error_memory = 0;
turn_error_last = 0;
dist_error_last = 0;

no_ball_loops = 0;

last_command = ""

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

        x_medium = int(x + w / 2)
        y_medium = int(y + h / 2)
    
    
    if size >= size_treshold:
        # we have a ball so we reset the no ball counter
        no_ball_loops = 0;
        
        distance = int(dist_to_size / size)
        dist_from_center = center - x_medium 
        dist_from_center = x_shift_to_mm * dist_from_center / (x_shift_cal / distance)

        alpha = math.degrees(math.atan2(dist_from_center , distance))
        
        # print(f"Alpha: {alpha}")

        cv2.line(frame, (x_medium, y_medium), (W // 2, H), (0, 255, 0), 2)
        
        cv2.putText(frame,str(int(size)), (10,10), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0), 2)
        cv2.putText(frame,str(distance), (10,30), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,0), 2)
        cv2.putText(frame,str(alpha), (10,50), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,0), 2)
    
        # cv2.line(frame, (x_medium, 0), (x_medium, H), (0, 255, 0), 2)
        # cv2.line(frame, (0, y_medium), (W, y_medium), (0, 255, 0), 2)
        
        # moving around
        # turn_error = center - x_medium;
        # dist_error = distance - target_distance;

        # # let's calulate the required turn based on distance from camera and the sizemove.
        

        # # for pid's
        # # turning
        # turn_error_memory += turn_error
        # turn_error_delta = turn_error - turn_error_last
        # turn_error_last = turn_error
        
        # # moving
        # dist_error_memory += dist_error
        # dist_error_delta = dist_error - dist_error_last
        # dist_error_last = dist_error

        
        # turn_angle = 0;
        # move_distance = 0;

        
        # # if abs(turn_error) > 30:
        # if True:
        #     turn_angle = int(Kp_angle * turn_error + Ki_angle * turn_error_memory + Kd_angle * turn_error_delta)            

        
        # # if abs(turn_angle) < 20:
        # if True:
        #     move_distance = math.ceil(Kp_dist * dist_error + Ki_dist * dist_error_memory + Kd_dist * dist_error_delta)

        # # print(move_distance, turn_angle)
        # if move_distance != 0 or turn_angle != 0:
        # adding some death zone
        
        move_distance = distance
        turn_angle = int(alpha / 10)

        if abs(move_distance) > dist_death_zone or abs(turn_angle) > angle_death_zone : 
            
            if abs(turn_angle) < angle_death_zone:
                # if we dont turn we can move
                # message = f'<1,{move_distance }, 0,500>'
                message = f'<0,0, 0,500>'
            else:
                # if we need to turn we just turn
                message = f'<1,0, {turn_angle} ,250>'
                
            print(message.encode('utf-8'))

            last_was_zero = False
            # if loop > loopdelay and last_command != message:    
            if loop > loopdelay:    
                loop = 0
                try:
                    ser.write(message.encode('utf-8'))
                    while (True):
                        response = str(ser.readline())
                        if "command" in response:
                            # print("Command confirmed!")
                            break
                except:
                    pass

            last_command = message

        else:
            if not last_was_zero: 
                message = f'<8,0,0,0>'
                print(message.encode('utf-8'))
                last_was_zero = True

                if loop > loopdelay:    
                    loop = 0
                    try:
                        ser.write(message.encode('utf-8'))
                    except:
                        pass

    else:
            # if we didn't already - we send stop command
            # print(f"No ball for {no_ball_loops} loops")
            if not last_was_zero: 
                message = f'<8,0,0,0>'
                print(message.encode('utf-8'))
                last_was_zero = True

                if loop > loopdelay:    
                    loop = 0
                    try:
                        ser.write(message.encode('utf-8'))
                    except:
                        pass

            #  counting for how many loopswe don't see ball
            no_ball_loops += 1
            if (no_ball_loops > 100):
                print("Gimme Ball!!!!")
            
    
    cv2.imshow("Frame", frame)
    
    loop += 1;

    key = cv2.waitKey(1)
    
    if key == 27:
        break
    
cap.release()
cv2.destroyAllWindows()
