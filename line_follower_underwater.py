import time
import cv2 as cv
import pymurapi
from math import atan2, pi, degrees
import numpy as np

print (cv.__version__)

auv = pymurapi.mur_init()

# коэффициент глубины
k_d = 10
target_depht = 3

min_color = (130, 130, 130)
max_color = (160, 255, 255)

# коэффициент курса
k_y = 0.8
last_error = 0
summa_error = 0

last_error_depth = 0
flag = False

def p_depth(target_depth):
    global last_error_depth
    # текущая глубина
    current_depth = auv.get_depth()
    # ошибка - это
    error = current_depth - target_depth
    power = error * k_d + (error - last_error_depth) * 3
    # подача тяги на движители
    auv.set_motor_power(2, clamp(power, -100, 100))
    auv.set_motor_power(3, clamp(power, -100, 100))
    last_error_depth = error
    #time.sleep(0.02)

def p_yaw(speed, error):
    global last_error
    global summa_error
    # ошибка - это
    power = error*0.5 + (error - last_error) * 1.2 + summa_error * 0.001
    # подача тяги на движители
    auv.set_motor_power(0, int(clamp(speed - power, -30, 30)))
    auv.set_motor_power(1, int(clamp(speed + power, -30, 30)))
    last_error = error
    summa_error = summa_error * 0.5 + error
    time.sleep(0.02)

def clamp(x, minX, maxX):
    if x < minX:
        return minX
    if x > maxX:
        return maxX
    return x

def image_process(image):
    global flag
    error = 0
    image = cv.GaussianBlur(image, (5, 5), -1)
    gray_picture = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    _, dst = cv.threshold(gray_picture, 200, 255,  cv.THRESH_BINARY_INV)
    th3 = cv.adaptiveThreshold(gray_picture,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,\
    cv.THRESH_BINARY_INV,5,2)
    image_search_circle = th3[120:240, 0:320]
    cv.imshow('image_search_circle', image_search_circle)
    rows = image_search_circle.shape[0] / 8
    count_circle = []
    count_circle = cv.HoughCircles(image_search_circle,
                                   cv.HOUGH_GRADIENT, 
                                   1,  
                                   rows, 
                                   param1= 100, #нижние границы бинаризции
                                   param2= 30, #верхние границы бинаризции
                                   minRadius= 0,
                                   maxRadius= 100)
    pixelL = np.argmax(th3[40])
    a = th3[40][::-1]
    pixelR = len(a) - np.argmax(a)
    cv.circle(original,(pixelL, 40), 5, (0, 0, 255), -1)
    cv.circle(original,(pixelR, 40), 5, (0, 255, 0), -1)
    error = error + (160 - (pixelL + pixelR)/2)
    pixelL = np.argmax(th3[120])
    a = th3[120][::-1]
    pixelR = len(a) - np.argmax(a)
    cv.circle(original,(pixelL, 120), 5, (0, 0, 255), -1)
    cv.circle(original,(pixelR, 120), 5, (0, 255, 0), -1)
    error = error + (160 - (pixelL + pixelR)/2)
    try:
        c_c = len(count_circle[0])
    except:
        c_c = 0
    
    if c_c > 0 and flag == False:
        auv.drop()
        flag = True
    if c_c == 0:
        flag = False
    print ('c',c_c,'e', error)
    cv.imshow('th3', th3)
    return  error

depht = 0
arr_depth = [0 for i in range(1000)]
while True:
    p_depth(target_depht)
    arr_depth.pop(0)
    arr_depth.append(auv.get_depth())
    #print(arr_depth)
    depht = np.mean(arr_depth)
    print(depht)
    if target_depht * 0.95<depht < target_depht*1.05:
        break


while True:
    image = auv.get_image_bottom()
    original = image.copy()
    error = image_process(image)
    cv.imshow('2', original)
    p_depth(3)
    p_yaw(10,error)

    if cv.waitKey(1) == 27:
        break
