# Ball Balancing Bot
# Date : 28.09.2017
# Code by : Anuradha Gunawardhana
#         : Dinuka Bandara
#
# Description : This code use OpenCV library to track the color of a ball on frames of a webcam feed and get the co-ordinates 
#               Then the code will calculate the position where the ball should move and send servo signals to arduino
#               The servos are contolled by two PID control loops
#               Python code and the arduino communicates over the "firmata" protocol
#
# Usage :       To use presets
#                   Change "usePreset" variable to "True"
#                   change "presetHSV("") to preset color
#                   To set a color as preset add its HSV values and name into presetHSV() method

from pyfirmata import Arduino, util
import cv2
import numpy as np
import time
import math

# set point values when start previous x -319, y-239
x = 319
y = 239

# Co-ordinates of the point where the ball should stay still
goalX = 319
goalY = 239

# Boundaries of the plate
xMin = 150
xMax = 490
yMin = 60
yMax = 420

# Ball width and height for ignoring other objects except the laser pointer 
ball_w = 90
ball_h = 90
pointer_change = False

#move on a circle and figure 8 date
radius_8 = 70
c_radius = 90
c_speed = 0.05
fig8_speed = 0.005
t1=0
t2=0
circle_on = False 
fig8_on = False

# Xpoint(goal point) color
p_color  = (255, 0, 0)

# initial PID variables
xIntegrator = 0
xDerivator = 0
yIntegrator = 0
yDerivator = 0
x_prevTime = time.time()
y_prevTime = time.time()

# Define the Arduino board com port
board = Arduino('COM9')

# Select servo pin (digital:PIN;servo)
servo_1 = board.get_pin('d:5:s')   # x-axis
servo_2 = board.get_pin('d:6:s')   # y-axis

# initial servo values
servo_1.write(90)
servo_2.write(90)

cap = cv2.VideoCapture(0)   # 1 is the external video source
# Set cap width and height
cap.set(3, 640)
cap.set(4, 480)

hsv_label = ['H_min', 'H_max', 'S_min', 'S_max', 'V_min', 'V_max']
hsv_value = [0, 179, 0, 255, 0, 255]

usePreset = True   # Use this for preset colour values(True or False)
presetDone = False

kernel_erode = np.ones([3,3], np.uint8)
kernel_dilate = np.ones([8,8], np.uint8)

# Trackbar call back function
def nothing(x):
    pass

# Store colour preset values
def presetHSV(req):
    global presetDone
    # [H_min, H_max, S_min, S_max, V_min, V_max]
    color = ['green', 'blue', 'white','blueBall']
    color_value = [[50, 78, 85, 154, 107, 145],     # green
                   [104, 137, 103, 226, 50, 168],   # blue
                   [0, 0, 0, 0, 255, 255],          # Track white
                   [107, 128, 154, 255, 80, 255]]   # blue Ball
        
    for i in color:
        v = color.index(i)
        if i == str(req):
            for u in range(6):
                hsv_value[u] = color_value[v][u]
    presetDone = True
    
def getHSV():
    
    for i in range(6):
        hsv_value [i] = cv2.getTrackbarPos(hsv_label[i], 'HSV_Select')
        
def createTrackbars():    
    #['H_min', 'H_max', 'S_min', 'S_max', 'V_min', 'V_max']
    for i in xrange(6):
        if i % 2 == 0:
            cv2.createTrackbar(hsv_label[i], 'HSV_Select', hsv_value[i], hsv_value[i+1], nothing)
        if i % 2 == 1:
            cv2.createTrackbar(hsv_label[i], 'HSV_Select', hsv_value[i], hsv_value[i], nothing)
    
if usePreset == False:
    cv2.namedWindow('HSV_Select', 1)
    cv2.resizeWindow('HSV_Select', 450, 310)
    createTrackbars()
    
# calculate PID values for current error
def calculatePID(setPoint, currentValue, axis, kp = 0.22, ki = 0.28, kd = 0.09, IvalMax = 100, IvalMin = -100):
    global xIntegrator, xDerivator,yIntegrator, yDerivator, x_prevTime, y_prevTime
    #global kp, ki, kd
    # Set point coordinates for the object to stay still
    #setPoint_x = 319
    #setPoint_y = 239
    
    if axis == 'x':
        D_value=0
        # Error calculation
        error = setPoint - currentValue
        #print "xerror:",error
        # Calculation for P value
        P_value = kp*error
        #print "P_value = " + str(P_value)

        crntTime = time.time()
        dt = crntTime - x_prevTime
        #print "xdt - ",dt
        # Calculation for I value
        xIntegrator += error*dt
        I_value = ki*xIntegrator
        
        
        if xIntegrator > IvalMax:
            xIntegrator = IvalMax
           #xIntegrator = 0
        if xIntegrator < IvalMin:
            xIntegrator = IvalMin
            #xIntegrator = 0
        #print "X_I_value=" + str(I_value)+"_"
        
        if dt>0:
            D_value = kd*(error - xDerivator)/dt
            #print "D_value = " + str(D_value)
            xDerivator = error
        #print "xD_value = " + str(D_value)
        PID = P_value + I_value + D_value
     
        
        #print "xPID value = " + str(PID)
        x_prevTime = time.time()
        return PID

    
    if axis == 'y':
        D_value = 0
        # Error calculation
        error = setPoint - currentValue
        #print "yerror:",error
        
        # Calculation for P value
        P_value = kp*error
        #print "P_value = " + str(P_value)
        
        crntTime = time.time()
        dt = crntTime - y_prevTime
        #print "ydt - " , dt
        # Calculation for I value
        yIntegrator += error*dt
        I_value = ki*yIntegrator
        
        
        if yIntegrator > IvalMax:
            yIntegrator = IvalMax
            #yIntegrator = 0
        if yIntegrator < IvalMin:
            yIntegrator = IvalMin
            #yIntegrator = 0

        if dt > 0:
            D_value = kd*(error - yDerivator)/dt
            #print "D_value = " + str(D_value)
            yDerivator = error
        #print "yD_value = " + str(D_value)
        
        PID = P_value + I_value + D_value
        #print "yPID value = " + str(PID)
        y_prevTime = time.time()
        return PID

while(1):
    # for circle and figure 8 path calculation
    if circle_on:
        p1 = 319 + int(c_radius*math.sin(t1))
        q1 = 239 + int(c_radius*math.cos(t1))
        goalX = p1
        goalY = q1
        t1 = c_speed + t1 
        
    if fig8_on:
        p2 = 319+int(2*radius_8*math.cos(2*math.pi*t2)*math.sqrt(math.cos(2*math.pi*t2)**2))
        q2 = 239+int(2*radius_8*math.cos(2*math.pi*t2)*math.sin(2*math.pi*t2))
        goalX = p2
        goalY = q2
        t2 = fig8_speed + t2

    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    if usePreset == True and presetDone == False:
        presetHSV('white')        # Put preset color value
    elif usePreset == False:
        getHSV()
    
    max_hsv = np.array([hsv_value[1], hsv_value[3], hsv_value[5]])
    min_hsv = np.array([hsv_value[0], hsv_value[2], hsv_value[4]])
    
    mask = cv2.inRange(hsv_frame, min_hsv, max_hsv)
    
    # We only need to check frame and the mask so we set the first two inputs to frame
    res = cv2.bitwise_and(frame, frame, mask = mask)
    
    # Use erode and dilate methods multiple times to get a better image 
    erode = cv2.erode(mask, kernel_erode)
    erode = cv2.erode(erode, kernel_erode)
    dilate = cv2.dilate(erode, kernel_dilate)
    dilate = cv2.dilate(dilate, kernel_dilate)
    
    _, contours, hierarchy = cv2.findContours(dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours
    cv2.putText(frame, 'X', (goalX , goalY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (p_color[0], p_color[1], p_color[2]), 1, cv2.LINE_AA)
    if len(cnt) >0 :
        for c in cnt:
            M = cv2.moments(c)
            xc = int(M["m10"] / M["m00"])
            yc = int(M["m01"] / M['m00'])
            
            cv2.drawContours(frame, [c], -1, (255, 50,100), 2)
            cv2.circle(frame, (xc, yc), 3, (0, 0, 0), -1)
            cv2.putText(frame, '(' + str(xc) + ',' + str(yc) + ')', (xc + 20,  yc + 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 50,100), 1, cv2.LINE_AA)
            #cv2.putText(frame, 'X', (goalX , goalY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            x11,y11,w,h = cv2.boundingRect(c)
            #print "w:H = ",w,h , "x,y=" ,x11,y11

            if 15<w and w<30 and 15<h and h<30 and xMin < x11 and x11<xMax and yMin < y11 and y11 < yMax:
                if pointer_change :
                    goalX = x11
                    goalY = y11

            else:
                if ball_w-20 <w and w<ball_w+20 and ball_h-20 <h and h < ball_h+20 :
                    x = xc 
                    y = yc  
    else :
        x = 319
        y = 239
    #print "s1value = " + str(s1Value) + " PIDxMin = " + str(PIDxMin) + " PIDxMax = " + str(PIDxMax)
    ##print "PIDvalue = " + str(calculatePID(goalX, x))
    div_x=80
    s1Value = calculatePID(goalX,x,'x')
    if s1Value > div_x:
        s1Value = div_x
    if s1Value < -div_x:
        s1Value = -div_x
    s1Value += 90
    
    div_y=80
    s2Value = calculatePID(goalY,y,'y')
    if s2Value > div_y:
        s2Value = div_y
    if s2Value < -div_y:
        s2Value = -div_y
    s2Value += 90
    
    servo_2.write(s1Value)  #### x  
    servo_1.write(s2Value)    ### y
    #print "y=s2value = " + str(s2Value) +", x=s1value = "+str(s1Value)
    
    cv2.imshow('Capture', frame)
    cv2.imshow('res', res)
        
    
    #cv2.imshow('Mask', mask)
    #cv2.imshow('Erosion and Dilation', dilate)        # if you want to show this comment out contours before

    prs_key=cv2.waitKey(2)
    
    if prs_key == 112:                           # press "p" to use the laser pointer
        pointer_change = not pointer_change             # else: center point
        circle_on = False
        fig8_on = False
        if pointer_change :
            p_color = (0, 255, 0)
        else:
            p_color = (255, 0, 0)
            goalX = 319
            goalY = 239

    elif prs_key == 114:                           # press 'r' move on a circle
        circle_on = not circle_on                       # else : center point
        pointer_change = False
        fig8_on = False
        if circle_on:
            p_color = (0, 0, 255)
        else:
            p_color = (255, 0, 0)
            goalX = 319
            goalY = 239

    elif prs_key== 56:                           # press '8' to move on figure
        fig8_on = not fig8_on                          # else : center point
        pointer_change = False
        circle_on = False
        if circle_on:
            p_color = (255, 255, 0)
        else:
            p_color = (255, 0, 0)
            goalX = 319
            goalY = 239

    elif prs_key == 27:                           # press Escape to brake the loop
        break
        
cap.release()
cv2.destroyAllWindows()
#print 'Last run HSV values - ' + str(hsv_value)