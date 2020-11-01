# This is file for a ball search by Dz3
# it's a second revision solution for more smooth operations
# it's apart of the Dz3 robot project
# https://sites.google.com/view/myfluxcapacitor/dz3
# 
# by Tymacnjo in (crazy)2020


import cv2
import numpy as np
import serial
import math
import sys

# Trying to connect via ble-serial to the arduino part of Dz3
#  odpalenie BT com - na linux, z użyciem ble-serial dającego port /tmp/ttyBLE
ble_connected = True
try:
    ser = serial.Serial('/tmp/ttyBLE', timeout=1)
    ser.baudrate = 9600
except:
    ble_connected = False
    print("Serial BT port error...")

#  

# Turning on the webcam
cap = cv2.VideoCapture(2)

# Setting the resolution to work with
W, H = (1280 // 2, 720 // 2)
cap.set(3, W)
cap.set(4, H)

# default is to try showthe video window
show_window = True

# trying to use the camera
try:
    _, frame = cap.read()
    rows, cols, _ = frame.shape

except:
    # if we cant join to camera - let's do a scanfor such devices
    cams_test = 10
    cameras = []
    for i in range(0, cams_test):
        cap = cv2.VideoCapture(i)
        test, frame = cap.read()
        print("i : "+str(i)+" /// result: "+str(test))
        if test:
            cameras.append(i)

    # stopping further execution
    print("Detected cameras id's:")
    print(cameras)
    
    if len(cameras) > 0:
        # we try to use on of detected cameras:
        # Turning on the webcam

        cam_id = cameras[0]
        cap = cv2.VideoCapture(cam_id)

        # Setting the resolution to work with
        W, H = (1280 // 2, 720 // 2)
        cap.set(3, W)
        cap.set(4, H)
        try:
            _, frame = cap.read()
            rows, cols, _ = frame.shape
        except:
            # if we fail - exitthe code
            sys.exit("Wrong camera definition, please check the device number")

# As we have now picture, let's calulate the the frame diag size 
diag = math.sqrt(W**2 + H**2)

# and some usefull variables 
x_medium = int(cols / 2)
center = int(cols / 2)
center_y = int(rows / 2) 
position = 90 # degrees
loopdelay = 0
loop = 0
boring_loops = 0
size = 0
size_treshold = 5

# stuff for physical rd ball calibration
# this may vary depending on the used object and camera
red_ball_size = 105 #[mm] sizeof ball in real life - diameter
frame_size_at_250mm = 43 #[percent od screen diag] size in frame when is 250mm from lens in % of diag 
distance_0 = 260 # [mm] distance used for a left right shift calibration
dist_to_size = distance_0 * frame_size_at_250mm # recalculation constans from calibration

# the variable of distance initiated as zero
distance = 0 

# stetup of the PID controllers for move and turn.
# turn part:
angle_death_zone = 1
Kp_angle = 0.05
Ki_angle = 0#0.0001
Kd_angle = 0#0.01

# distance part:
target_distance = 550 #[mm]
dist_death_zone = 10 #[cm]
Kp_dist = 0.05
Ki_dist = 0#0.00005
Kd_dist = 0#0.01

# stuff for PID
# variables required for my simplified PID algorithm 
turn_error_memory = 0;
dist_error_memory = 0;
turn_error_last = 0;
dist_error_last = 0;

# Camera servo control variables (up/down move)
up_error = 0;
up_error_death_zone = 6
Kp_up = 0.01
angle_up_previous = 0
angle_up = 30;

# flag to track if last loop was stopping Dz3
last_was_zero = False
# for how long there was no ball in frame detected 
no_ball_loops = 0;
# memorizing last command
last_command = ""

last_angle = 0
last_distance = 0

# flag for the second servo - the hand one
hand_up = False


# main loop 
while True:
    # grab the frame
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # red color filtering 
    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)

    # getting the detected counturs 
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    
    # taking care only for the biggest detected object
    if len(contours) > 0:
        cnt = contours[0]
        (x, y, w, h) = cv2.boundingRect(cnt)

        # calculating the size
        size = 100 * math.sqrt(w**2 + h**2) / diag
        distance = int(dist_to_size / size) 
        
        # calculating the position
        x_medium = int(x + w / 2)
        y_medium = int(y + h / 2)
    
    # if the detected color contur is biger then defined treshold we do next stuff
    if size >= size_treshold:
        # we have a ball so we reset the no ball counter
        no_ball_loops = 0;

        # drawing lines from bottom center to the detected ball
        cv2.line(frame, (x_medium, 0), (x_medium, H), (0, 255, 0), 2)
        cv2.line(frame, (0, y_medium), (W, y_medium), (0, 255, 0), 2)
        
        # moving around - calculating the distance of the ball from desired location (middle)
        turn_error = center - x_medium;
        dist_error = distance - target_distance;
        up_error = center_y - y_medium;

        # for pid's
        # turning
        turn_error_memory += turn_error
        turn_error_delta = turn_error - turn_error_last
        turn_error_last = turn_error
        
        # moving
        dist_error_memory += dist_error
        dist_error_delta = dist_error - dist_error_last
        dist_error_last = dist_error

        # resseting the PID outpuds before calculations
        turn_angle = 0;
        move_distance = 0;

        #experiment with camera on servo
        # movingthe camera up/down to follow the ball 
        
        # if abs(up_error) > up_error_death_zone:
        #     angle_up += Kp_up * up_error
        #     angle_up = int(max(0, min(90, angle_up)))
            
        #     if angle_up_previous != angle_up:
        #         message = f'<41,{angle_up}, 0, 0>'
        #         print(message)
        #         try:
        #             ser.write(message.encode('utf-8'))
        #         except:
        #             pass
        # angle_up_previous = angle_up
        
        # # rise hand as yousee the ball!!!
        # if not hand_up:
        #     hand_up = True;
        #     message = f'<42,140, 0, 0>'
        #     print(message)
        #     try:
        #         ser.write(message.encode('utf-8'))
        #     except:
        #         pass

        # calculating PID for turning
        turn_angle = int(Kp_angle * turn_error + Ki_angle * turn_error_memory + Kd_angle * turn_error_delta)            

        # calculating PID for moving
        move_distance = math.ceil(Kp_dist * dist_error + Ki_dist * dist_error_memory + Kd_dist * dist_error_delta)

        # experimental - loking for a way to get rid of the hickup-moves
        # the idea is - if we want to move less than the last time, let's just don't sent another
        # move command.
        
        direction = True
        if (move_distance < 0 and last_distance > 0) or (move_distance > 0 and last_distance <0):
            direction = False 
         
        if abs(move_distance) < abs(last_distance) and direction:
            move_distance = 0
        
        
        # adding some general death zone
        if abs(move_distance) > dist_death_zone or abs(turn_angle) > angle_death_zone : 
            message = f'<1,{move_distance }, {turn_angle},1600>'
            
            # if abs(turn_angle) < angle_death_zone:
                # # if we dont turn we can move
                # message = f'<1,{move_distance }, {turn_angle},800>'
            # else:
                # # if we need to turn we just turn
                # message = f'<1,{move_distance // 10}, {turn_angle} ,2000>'
                
            # printing the sent message - for debug
            print(message.encode('utf-8'))

            # resseting the flag that we didn't move 
            last_was_zero = False
            
            # this allows to skip sending messages for given numbers of loop
            if loop >= loopdelay and last_command != message:    
                loop = 0
                try:
                    ser.write(message.encode('utf-8'))
                    ## the below allows to wait for a respond - but it didn't work well
                    ## and created choppy motions
                    # while (True):
                    #     response = str(ser.readline())
                    #     if "command" in response:
                    #         # print("Command confirmed!")
                    #         break
                except:
                    pass

            last_command = message
            last_angle= turn_angle
            last_distance = move_distance

        else:
            # if we don't need to move - we send the command to stop
            # new experimental idea for soft stop to move 1/5 of last move
            if not last_was_zero: 
                # message = f'<8,0,0,0>'
                message = f'<1,{last_distance // 2},{last_angle // 2},800>'
                print(message.encode('utf-8'))
                last_was_zero = True

                if loop > loopdelay:    
                    loop = 0
                    try:
                        ser.write(message.encode('utf-8'))
                        pass
                    except:
                        pass

    else:   
            no_ball_loops += 1
            if (no_ball_loops < 100) and  (not last_angle == 0 or last_distance ==0):
                # we turn as last - to check if the ball roll out of frame
                message = f'<1,{last_distance},{last_angle},1250>'
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

                boring_loops += 1;
                print("Gimme Ball!!!!")
                
                if boring_loops > 30:
                    boring_loops = 0
                    # if we wait long we search around
                    message = f'<1,0,360,500>'
                    print(message.encode('utf-8'))
                    last_was_zero = True
                    try:
                        ser.write(message.encode('utf-8'))
                    except:
                        pass


            # if hand_up:
            #     hand_up = False;
            #     message = f'<42,5, 0, 0>'
            #     print(message)
            #     try:
            #         ser.write(message.encode('utf-8'))
            #     except:
            #         pass

            # if 40 < angle_up or angle_up < 25:  
            #     angle_up = 30;
            #     message = f'<41,30,0,0>'
            #     print("cam level")
            #     try:
            #         ser.write(message.encode('utf-8'))
            #     except:
            #         pass


    # preparing the on screen displays
    if show_window:
        try:
            cv2.putText(frame,str(int(size)), (10,10), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
            cv2.putText(frame,str(distance), (10,25), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
            cv2.putText(frame,str(up_error), (10,40), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
            cv2.imshow("Frame", frame)
        except:
            show_window= False
    
    # adding the loop counter
    loop += 1;

    # reading keyboard
    key = cv2.waitKey(1)
    # if ESC or q is pressed we end execution 
    if key == 27 or key==ord('q'):
        break

# R<eleasing resources 
cap.release()
cv2.destroyAllWindows()
