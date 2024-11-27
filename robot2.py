# -*- coding: utf-8 -*-

import platform
import numpy as np
import argparse
import cv2
import serial
import time
import sys
from threading import Thread, Lock
import csv
import math

# 전역 변수 초기화
X_255_point = 0
Y_255_point = 0
X_Size = 0
Y_Size = 0
Area = 0
Angle = 0
#-----------------------------------------------
Top_name = 'mini CTS5 setting'
hsv_Lower = 0
hsv_Upper = 0

hsv_Lower0 = 0
hsv_Upper0 = 0

hsv_Lower1 = 0
hsv_Upper1 = 0

#----------- 
color_num = [0, 1, 2, 3, 4]
    
h_max = [255, 65, 196, 111, 110]
h_min = [55, 0, 158, 59, 74]
    
s_max = [162, 200, 223, 110, 255]
s_min = [114, 140, 150, 51, 133]
    
v_max = [77, 151, 239, 156, 255]
v_min = [0, 95, 104, 61, 104]
    
min_area = [50, 50, 50, 10, 10]

now_color = 0
serial_use = 1

serial_port = None
Temp_count = 0
Read_RX = 0

mx, my = 0, 0

threading_Time = 5/1000.

Config_File_Name = 'Cts5_v1.dat'

# 멀티스레딩을 위한 변수
frame = None
radius = 0
X, Y = 0, 0
running = True
frame_lock = Lock()
msg_one_view = 0

# 골프공 검색을 위한 변수
searching = False
search_direction = None
search_step = 0
max_steps = 9  # 10도 * 9 = 90도
search_attempts = 0  # 탐색 시도 횟수

# 시리얼 포트 명령어 정의 (사용자 확인 필요)
LEFT_TURN_10_GOLF = 4   # 왼쪽턴10_골프
RIGHT_TURN_10_GOLF = 6  # 오른쪽턴10_골프
RIGHT_TURN_45_GOLF = 24  # 오른쪽턴45_골프

#-----------------------------------------------

def nothing(x):
    pass

#-----------------------------------------------
def create_blank(width, height, rgb_color=(0, 0, 0)):

    image = np.zeros((height, width, 3), np.uint8)
    color = tuple(reversed(rgb_color))
    image[:] = color

    return image
#-----------------------------------------------
def draw_str2(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 255), lineType=cv2.LINE_AA)
#-----------------------------------------------
def draw_str3(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), lineType=cv2.LINE_AA)
#-----------------------------------------------
def draw_str_height(dst, target, s, height):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, height, (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, height, (255, 255, 255), lineType=cv2.LINE_AA)
#-----------------------------------------------
def clock():
    return cv2.getTickCount() / cv2.getTickFrequency()
#-----------------------------------------------

def Trackbar_change(now_color):
    global hsv_Lower, hsv_Upper
    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])

#-----------------------------------------------
def Hmax_change(a):
    h_max[now_color] = cv2.getTrackbarPos('Hmax', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Hmin_change(a):
    h_min[now_color] = cv2.getTrackbarPos('Hmin', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Smax_change(a):
    s_max[now_color] = cv2.getTrackbarPos('Smax', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Smin_change(a):
    s_min[now_color] = cv2.getTrackbarPos('Smin', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Vmax_change(a):
    v_max[now_color] = cv2.getTrackbarPos('Vmax', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def Vmin_change(a):
    v_min[now_color] = cv2.getTrackbarPos('Vmin', Top_name)
    Trackbar_change(now_color)
#-----------------------------------------------
def min_area_change(a):
    min_area[now_color] = cv2.getTrackbarPos('Min_Area', Top_name)
    if min_area[now_color] == 0:
        min_area[now_color] = 1
        cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])
    Trackbar_change(now_color)
#-----------------------------------------------
def Color_num_change(a):
    global now_color, hsv_Lower, hsv_Upper
    now_color = cv2.getTrackbarPos('Color_num', Top_name)
    cv2.setTrackbarPos('Hmax', Top_name, h_max[now_color])
    cv2.setTrackbarPos('Hmin', Top_name, h_min[now_color])
    cv2.setTrackbarPos('Smax', Top_name, s_max[now_color])
    cv2.setTrackbarPos('Smin', Top_name, s_min[now_color])
    cv2.setTrackbarPos('Vmax', Top_name, v_max[now_color])
    cv2.setTrackbarPos('Vmin', Top_name, v_min[now_color])
    cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])

    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])
#----------------------------------------------- 
def TX_data(ser, one_byte):  # one_byte= 0~255
    ser.write(serial.to_bytes([one_byte]))  # python3
#-----------------------------------------------
def RX_data(serial):
    global Temp_count
    try:
        if serial.inWaiting() > 0:
            result = serial.read(1)
            RX = ord(result)
            return RX
        else:
            return 0
    except:
        Temp_count = Temp_count + 1
        print("Serial Not Open " + str(Temp_count))
        return 0
        pass
#-----------------------------------------------

#*************************
# mouse callback function
def mouse_move(event, x, y, flags, param):
    global mx, my

    if event == cv2.EVENT_MOUSEMOVE:
        mx, my = x, y

# *************************
def RX_Receiving(ser):
    global receiving_exit, threading_Time

    global X_255_point
    global Y_255_point
    global X_Size
    global Y_Size
    global Area, Angle

    receiving_exit = 1
    while True:
        if receiving_exit == 0:
            break
        time.sleep(threading_Time)
        
        while ser.inWaiting() > 0:
            result = ser.read(1)
            RX = ord(result)
            print("RX=" + str(RX))

#************************
def hsv_setting_save():

    global Config_File_Name, color_num
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area
    
    try:
        saveFile = open(Config_File_Name, 'w')
        i = 0
        color_cnt = len(color_num)
        while i < color_cnt:
            text = str(color_num[i]) + ","
            text = text + str(h_max[i]) + "," + str(h_min[i]) + ","
            text = text + str(s_max[i]) + "," + str(s_min[i]) + ","
            text = text + str(v_max[i]) + "," + str(v_min[i]) + ","
            text = text + str(min_area[i])  + "\n"
            saveFile.writelines(text)
            i = i + 1
        saveFile.close()
        print("hsv_setting_save OK")
        return 1
    except:
        print("hsv_setting_save Error~")
        return 0

#************************
def hsv_setting_read():
    global Config_File_Name
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area

    try:
        with open(Config_File_Name) as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            i = 0
            
            for row in readCSV:
                color_num[i] = int(row[0])
                h_max[i] = int(row[1])
                h_min[i] = int(row[2])
                s_max[i] = int(row[3])
                s_min[i] = int(row[4])
                v_max[i] = int(row[5])
                v_min[i] = int(row[6])
                min_area[i] = int(row[7])
                
                i = i + 1
              
        csvfile.close()
        print("hsv_setting_read OK")
        return 1
    except:
        print("hsv_setting_read Error~")
        return 0

# **************************************************
def handle_robot_movement(X, Y, radius, serial_port):
    """
    Handles robot movement based on object position and size.
    """
    if radius > 0:
        # 공을 찾았을 때의 동작 수행
        if radius < 50:
            print("Command: 전진종종걸음_골프")
            TX_data(serial_port, 10)  # 전진종종걸음_골프
        elif 50 <= radius <= 94:
            TX_data(serial_port, 33)  # 안정화자세
            if X < 200:
                print("Command: 왼쪽옆으로70연속_골프")
                TX_data(serial_port, 14)  # 왼쪽옆으로70연속_골프
                time.sleep(1)
            elif X > 350:
                print("Command: 오른쪽옆으로70연속_골프")
                TX_data(serial_port, 13)  # 오른쪽옆으로70연속_골프
                time.sleep(1)
            else:
                print("Command: 전방하향60도")
                TX_data(serial_port, 36)  # 전방하향60도
                print("Command: 전진종종걸음_골프")
                TX_data(serial_port, 10)  # 전진종종걸음_골프
        elif 95 <= radius <= 97 and 200 <= X <= 350 and 180 <= Y <= 280:
            print("Command: 골프_왼쪽으로_샷")
            TX_data(serial_port, 2)  # 골프_왼쪽으로_샷
            TX_data(serial_port, 33)  # 안정화자세
        elif radius > 97:
            print("Command: 후진종종걸음_골프")
            TX_data(serial_port, 12)  # 후진종종걸음_골프
# **************************************************

# 카메라 처리 스레드
def camera_thread():
    global frame, radius, X, Y, running
    while running:
        # grab the current frame
        (grabbed, frame_local) = camera.read()

        if not grabbed:
            continue

        hsv = cv2.cvtColor(frame_local, cv2.COLOR_BGR2YUV)  # HSV => YUV
        hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
        hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])
        mask = cv2.inRange(hsv, hsv_Lower, hsv_Upper)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        with frame_lock:
            frame = frame_local.copy()

            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((X_local, Y_local), radius_local) = cv2.minEnclosingCircle(c)

                Area_local = cv2.contourArea(c) / min_area[now_color]
                if Area_local > 255:
                    Area_local = 255

                if Area_local > min_area[now_color]:
                    x4, y4, w4, h4 = cv2.boundingRect(c)
                    cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 0), 2)
                    #----------------------------------------
                    draw_str2(frame, (3, 35), 'Center: (%.1f, %.1f)' % (X_local, Y_local))
                    draw_str2(frame, (3, 55), 'Radius: %.1f' % radius_local)
                    #----------------------------------------
                    
                    X_Size = int((255.0 / W_View_size) * w4)
                    Y_Size = int((255.0 / H_View_size) * h4)
                    X_255_point = int((255.0 / W_View_size) * X_local)
                    Y_255_point = int((255.0 / H_View_size) * Y_local)
                else:
                    X_local = 0
                    Y_local = 0
                    radius_local = 0

                X = X_local
                Y = Y_local
                radius = radius_local
            else:
                X = 0
                Y = 0
                radius = 0

        time.sleep(0.01)  # 카메라 처리 간격

# 로봇 제어 스레드
def robot_thread():
    global X, Y, radius, serial_port, running
    global searching, search_direction, search_step, max_steps, search_attempts
    searching = False
    search_direction = None
    search_step = 0
    max_steps = 9  # 10도 * 9 = 90도
    search_attempts = 0  # 탐색 시도 횟수 초기화

    while running:
        with frame_lock:
            X_local = X
            Y_local = Y
            radius_local = radius

        if radius_local > 0:
            # 공을 찾았을 때
            if searching:
                print("Ball found during search.")
                searching = False
                search_attempts = 0  # 탐색 시도 횟수 초기화
            handle_robot_movement(X_local, Y_local, radius_local, serial_port)
        else:
            # 공을 찾지 못했을 때
            if not searching:
                if search_attempts == 0:
                    print("Starting search sequence.")
                searching = True
                search_direction = 'left'
                search_step = 0
            if searching:
                if search_direction == 'left':
                    if search_step < max_steps:
                        print("Searching left: Step {}".format(search_step + 1))
                        # 몸 왼쪽으로 10도 회전
                        TX_data(serial_port, LEFT_TURN_10_GOLF)  # 왼쪽턴10_골프
                        time.sleep(1)  # 움직임 후 딜레이
                        search_step += 1
                    else:
                        # 왼쪽 검색 완료, 원위치로 복귀
                        print("Returning to center from left search.")
                        # 오른쪽으로 45도 회전 2번
                        TX_data(serial_port, RIGHT_TURN_45_GOLF)
                        time.sleep(1)
                        TX_data(serial_port, RIGHT_TURN_45_GOLF)
                        time.sleep(1)
                        search_direction = 'right'
                        search_step = 0
                elif search_direction == 'right':
                    if search_step < max_steps:
                        print("Searching right: Step {}".format(search_step + 1))
                        # 몸 오른쪽으로 10도 회전
                        TX_data(serial_port, RIGHT_TURN_10_GOLF)
                        time.sleep(1)  # 움직임 후 딜레이
                        search_step += 1
                    else:
                        # 오른쪽 검색 완료
                        print("Completed search sequence.")
                        searching = False
                        search_attempts += 1  # 탐색 시도 횟수 증가
                        if search_attempts == 1:
                            # 첫 번째 탐색 완료 후, 29 전송 및 재탐색
                            print("Sending 29 to serial_port, restarting search.")
                            TX_data(serial_port, 29)
                            time.sleep(1)
                            searching = True
                            search_direction = 'left'
                            search_step = 0
                        elif search_attempts == 2:
                            # 두 번째 탐색 완료 후, 31 전송 및 재탐색
                            print("Sending 31 to serial_port, restarting search.")
                            TX_data(serial_port, 31)
                            time.sleep(1)
                            searching = True
                            search_direction = 'left'
                            search_step = 0
                        else:
                            # 모든 탐색 시도 완료
                            print("All search attempts completed. Could not find the ball.")
                            # 필요한 경우 추가 동작 수행 가능
                            search_attempts = 0  # 탐색 시도 횟수 초기화
        time.sleep(0.1)

# **************************************************
# **************************************************
if __name__ == '__main__':

    #-------------------------------------
    print("-------------------------------------")
    print("(2020-1-20) mini CTS5 Program.  MINIROBOT Corp.")
    print("-------------------------------------")
    print("")
    os_version = platform.platform()
    print(" ---> OS " + os_version)
    python_version = ".".join(map(str, sys.version_info[:3]))
    print(" ---> Python " + python_version)
    opencv_version = cv2.__version__
    print(" ---> OpenCV  " + opencv_version)
    
    #-------------------------------------
    #---- user Setting -------------------
    #-------------------------------------
    W_View_size = 640  # 320  # 640
    # H_View_size = int(W_View_size / 1.777)
    H_View_size = int(W_View_size / 1.333)

    BPS = 4800  # 4800,9600,14400,19200,28800,57600,115200
    serial_use = 1
    now_color = 0
    View_select = 0
    #-------------------------------------
    print(" ---> Camera View: " + str(W_View_size) + " x " + str(H_View_size))
    print("")
    print("-------------------------------------")
    
    #-------------------------------------
    try:
        hsv_setting_read()
    except:
        hsv_setting_save()
        
    #-------------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
                    help="max buffer size")
    args = vars(ap.parse_args())

    img = create_blank(320, 100, rgb_color=(0, 0, 255))
    
    cv2.namedWindow(Top_name)
    cv2.moveWindow(Top_name, 0, 0)
    
    cv2.createTrackbar('Hmax', Top_name, h_max[now_color], 255, Hmax_change)
    cv2.createTrackbar('Hmin', Top_name, h_min[now_color], 255, Hmin_change)
    cv2.createTrackbar('Smax', Top_name, s_max[now_color], 255, Smax_change)
    cv2.createTrackbar('Smin', Top_name, s_min[now_color], 255, Smin_change)
    cv2.createTrackbar('Vmax', Top_name, v_max[now_color], 255, Vmax_change)
    cv2.createTrackbar('Vmin', Top_name, v_min[now_color], 255, Vmin_change)
    cv2.createTrackbar('Min_Area', Top_name, min_area[now_color], 255, min_area_change)
    cv2.createTrackbar('Color_num', Top_name, color_num[now_color], 4, Color_num_change)

    Trackbar_change(now_color)

    draw_str3(img, (15, 25), 'MINIROBOT Corp.')
    draw_str2(img, (15, 45), 'space: Fast <=> Video and Mask.')
    draw_str2(img, (15, 65), 's, S: Setting File Save')
    draw_str2(img, (15, 85), 'Esc: Program Exit')
    
    cv2.imshow(Top_name, img)
    #---------------------------
    if not args.get("video", False):
        camera = cv2.VideoCapture(0)
    else:
        camera = cv2.VideoCapture(args["video"])
    #---------------------------
    camera.set(3, W_View_size)
    camera.set(4, H_View_size)
    camera.set(5, 60)
    time.sleep(0.5)
    #---------------------------
    (grabbed, frame) = camera.read()
    draw_str2(frame, (5, 15), 'X_Center x Y_Center =  Area')
    draw_str2(frame, (5, H_View_size - 5), 'View: %.1d x %.1d.  Space: Fast <=> Video and Mask.'
                  % (W_View_size, H_View_size))
    draw_str_height(frame, (5, int(H_View_size/2)), 'Fast operation...', 3.0)
    mask = frame.copy()
    cv2.imshow('mini CTS5 - Video', frame)
    cv2.imshow('mini CTS5 - Mask', mask)
    cv2.moveWindow('mini CTS5 - Mask', 322 + W_View_size, 36)
    cv2.moveWindow('mini CTS5 - Video', 322, 36)
    cv2.setMouseCallback('mini CTS5 - Video', mouse_move)

    #---------------------------
    if serial_use != 0:  # python3
        BPS = 4800  # 4800,9600,14400,19200,28800,57600,115200
        #---------local Serial Port : ttyS0 --------
        #---------USB Serial Port : ttyAMA0 --------
        serial_port = serial.Serial('/dev/ttyS0', BPS, timeout=0.01)
        serial_port.flush()  # serial cls
        time.sleep(0.5)
    
        serial_t = Thread(target=RX_Receiving, args=(serial_port,))
        serial_t.daemon = True
        serial_t.start()
        
    # First -> Start Code Send 
    TX_data(serial_port, 250)
    TX_data(serial_port, 250)
    TX_data(serial_port, 250)
    
    old_time = clock()

    View_select = 0
    # -------- Main Loop Start --------

    # 카메라 및 로봇 스레드 시작
    cam_thread = Thread(target=camera_thread)
    rob_thread = Thread(target=robot_thread)
    cam_thread.start()
    rob_thread.start()

    try:
        while True:
            with frame_lock:
                if frame is not None:
                    if View_select == 1:
                        # Debug 모드에서만 영상과 마스크를 표시
                        cv2.imshow('mini CTS5 - Video', frame)
            key = 0xFF & cv2.waitKey(1)
            
            if key == 27:  # ESC  Key
                break
            elif key == ord(' '):  # spacebar Key
                if View_select == 0:
                    View_select = 1
                else:
                    View_select = 0
            elif key == ord('s') or key == ord('S'):  # s or S Key:  Setting values Save
                hsv_setting_save()
                msg_one_view = 1
    finally:
        # cleanup the camera and close any open windows
        running = False
        cam_thread.join()
        rob_thread.join()
        receiving_exit = 0
        time.sleep(0.5)
        
        camera.release()
        cv2.destroyAllWindows()
