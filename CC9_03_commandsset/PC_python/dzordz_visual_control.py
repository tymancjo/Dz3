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
from time import sleep
from Dz3 import Dz3_library as dz
# import os
# import subprocess

# used blte.py from https://github.com/GunnarI/bluepy/tree/add-timeout-to-perhiperal
# as it has a timeout on connection available
import bluepy.btle as btle


# trying the system shell command execution
# to run the bluetooth connection
# BLE = subprocess.Popen("ble-serial -d 88:25:83:f0:fe:e6", shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
# print(BLE.communicate)
# # givingit a bit of time
# sleep(1)





# Trying to connect via ble-serial to the arduino part of Dz3
#  odpalenie BT com - na linux, z użyciem ble-serial dającego port /tmp/ttyBLE
ble_connected = True
BT_connected = False

try:
    ser = serial.Serial('/tmp/ttyBLE', timeout=1)
    ser.baudrate = 9600
except:
    ble_connected = False
    print("Serial BT port error...")
    print("Going to try direct connection to BT BLE...")

    try:
        BTEperihibal = btle.Peripheral("88:25:83:f0:fe:e6", timeout=3)
        print(f'BTE {BTEperihibal}')
        BTEservice = BTEperihibal.getServiceByUUID("0000ffe0-0000-1000-8000-00805f9b34fb")
        Dzordz = BTEservice.getCharacteristics()[0]
        BT_connected = True

    except:
        BT_connected = False


def BTsent(message, wait=False):
    """
    Sending command message to Dżordż
    """
    if ble_connected:
        try:
            ser.write(message.encode('utf-8'))
        except:
            print("Error sending over serial :(")
    
    elif BT_connected:
        Dzordz.write(bytes(message, "utf-8"))

        if wait:
            BTEperihibal.withDelegate(ReadDelegate())
            while BTEperihibal.waitForNotifications(0.1):
                pass
    else:
        print("No connection is available...")

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
            sys.exit("Wrong camera definition, please check the device number and connections")

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
PID_angle = dz.PID(0.05, 0, 0, x_medium, False)

# distance part:
target_distance = 350 #[mm]
dist_death_zone = 10 #[cm]
PID_distance = dz.PID(0.05, 0, 0,target_distance, False)

# Camera servo control variables (up/down move)
PID_camera = dz.PID(0.03, 0.0000005, 0.01, center_y, False)
up_error_death_zone = 6
angle_up = 30;
angle_up_previous = 30;

# flag to track if last loop was stopping Dz3
last_was_zero = False
# for how long there was no ball in frame detected 
no_ball_loops = 0;
search_loops = 0;

# memorizing last command
last_command = ""

last_angle = 0
last_distance = 0

# flag for the second servo - the hand one
hand_up = False

# main loop 
while True:
    try:
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

            # resseting the PID outpuds before calculations
            turn_angle = 0;
            move_distance = 0;

            # camera servo moving up and down 
            if abs(PID_camera.get_error(y_medium)) > up_error_death_zone:
                        
                angle_up += PID_camera.update(y_medium)
                angle_up = int(max(0, min(90, angle_up)))
                
                if angle_up_previous != angle_up:
                    message = f'<41,0,{angle_up}, 0>'
                    print(message)

                    BTsent(message)
                    # try:
                    #     ser.write(message.encode('utf-8'))
                    # except:
                    #     pass
                else:
                    pass        
            angle_up_previous = angle_up

            # calculating PID for turning
            turn_angle = int( PID_angle.update(x_medium) )
            
            # calculating PID for moving
            move_distance = math.ceil( PID_distance.update(distance) )
            
            direction = True
            if (move_distance < 0 and last_distance > 0) or (move_distance > 0 and last_distance <0):
                direction = False 
            
            if abs(move_distance) < abs(last_distance) and direction:
                move_distance = 0
            
            the_speed = 2500
            
            # adding some general death zone
            if abs(move_distance) > dist_death_zone or abs(turn_angle) > angle_death_zone : 
                #message = f'<1,{move_distance }, {turn_angle},{the_speed}>'
                
                if abs(turn_angle) < angle_death_zone:
                    # if we dont turn we can move
                    message = f'<1,{move_distance }, {turn_angle},{the_speed}>'
                else:
                    # if we need to turn we just turn
                    message = f'<1,{move_distance // 10}, {turn_angle} ,{the_speed}>'
                
                # printing the sent message - for debug
                print(message.encode('utf-8'))

                # resseting the flag that we didn't move 
                last_was_zero = False
                
                # this allows to skip sending messages for given numbers of loop
                if loop >= loopdelay and last_command != message:    
                    loop = 0
                    BTsent(message)
                    # try:
                    #     ser.write(message.encode('utf-8'))
                    # except:
                    #     pass

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
                        BTsent(message)
                        # try:
                        #     ser.write(message.encode('utf-8'))
                        #     pass
                        # except:
                        #     pass

        else:   
                no_ball_loops += 1

                if (no_ball_loops < 100) and  (not last_angle == 0):
                    # we turn as last - to check if the ball roll out of frame
                    # message = f'<1,{last_distance},{last_angle},1250>'
                    message = f'<1,0,{last_angle},1250>'
                    BTsent(message)
                    # try:
                    #     ser.write(message.encode('utf-8'))
                    # except:
                    #     pass
                    
                    # and following with camera up-down
                    # and making it smallerin each loop
                    # angle_up += ( 2 * Kp_up * previous_up_error * (100 - no_ball_loops) / 100  )
                    angle_up = PID_camera.update(PID_camera.last_current * (100 - no_ball_loops) / 100)
                    angle_up = int(max(0, min(90, angle_up)))
                
                    message = f'<41,0,{angle_up}, 0>'
                    print(message)
                    BTsent(message)
                    # try:
                    #     ser.write(message.encode('utf-8'))
                    # except:
                    #     pass
                        
                else:
                    # if we didn't already - we send stop command
                    if not last_was_zero: 
                        message = f'<8,0,0,0>'
                        print(message.encode('utf-8'))
                        last_was_zero = True

                        if loop > loopdelay:    
                            loop = 0
                            BTsent(message)
                            # try:
                            #     ser.write(message.encode('utf-8'))
                            # except:
                            #     pass

                    boring_loops += 1;
                    
                    if boring_loops > 30:
                        boring_loops = 0
                        search_loops += 1
                        
                        message = f'<41,0,30, 0>'
                        angle_up = 30;
                        last_was_zero = True
                        
                        print(message)
                        BTsent(message)
                        # try:
                        #     ser.write(message.encode('utf-8'))
                        # except:
                        #     pass
                            
                        # if we wait long we search around
                        if search_loops == 1:
                            
                            message = f'<1,0,360,500>'
                            print("looking around...")
                            print(message.encode('utf-8'))
                            BTsent(message)
                            # try:
                            #     ser.write(message.encode('utf-8'))
                            # except:
                            #     pass

                        elif search_loops == 5:
                            message = f'<1,0,-360,500>'
                            print("looking around...")
                            print(message.encode('utf-8'))
                            BTsent(message)
                            # try:
                            #     ser.write(message.encode('utf-8'))
                            # except:
                            #     pass

                        elif search_loops == 15:
                            message = f'<1,15,0,500>'
                            print("looking around... moving...")
                            print(message.encode('utf-8'))
                            BTsent(message)
                            # try:
                            #     ser.write(message.encode('utf-8'))
                            # except:
                            #     pass
                        
                        elif search_loops == 20:
                            message = f'<1,10,45,500>'
                            print("looking around... moving...")
                            print(message.encode('utf-8'))
                            BTsent(message)
                            search_loops = 0
                            # try:
                            #     ser.write(message.encode('utf-8'))
                            # except:
                            #     pass

        # preparing the on screen displays
        if show_window:
            try:
                cv2.putText(frame,str(int(size)), (10,10), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
                cv2.putText(frame,str(distance), (10,25), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
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
    
    except KeyboardInterrupt:
        print("")
        print("Releasing resources...")

        # Releasing resources 
        cap.release()
        cv2.destroyAllWindows()

        # And on exit we have the killing of the process 
        
        sys.exit("exiting to system... thank you!")


# Releasing resources 
cap.release()
cv2.destroyAllWindows()



