import cv2
import numpy as np
import tkinter as tk
import tkinter.filedialog
import os

def do_nothing(x):
    pass

cap = cv2.VideoCapture(4)
cv2.namedWindow("TrackBar")
cv2.createTrackbar("H_H", "TrackBar", 0, 255,do_nothing)
cv2.createTrackbar("H_L", "TrackBar", 0, 255,do_nothing)
cv2.createTrackbar("S", "TrackBar", 0, 255,do_nothing)
cv2.createTrackbar("V", "TrackBar", 0, 255,do_nothing)

def detect_pic(img,gray):
    
    height, width = img.shape[:2]
    
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV_FULL)
    h = hsv[:,:,0]
    s = hsv[:,:,1]
    v = hsv[:,:,2]
    H_H = cv2.getTrackbarPos('H_H','TrackBar')
    H_L = cv2.getTrackbarPos('H_L','TrackBar')
    S_value = cv2.getTrackbarPos('S','TrackBar')
    V_value = cv2.getTrackbarPos('V','TrackBar')
    H_HIGH_VALUE = max(H_H,H_L)
    H_LOW_VALUE = min(H_H,H_L)
    img_matt = np.zeros((height,width,3),np.uint8)
    img_matt[(h < H_HIGH_VALUE) & (h > H_LOW_VALUE) & (s > S_value)& (v > V_value)] = 255
    cv2.imshow('gray',img_matt)
    
    img_gray = cv2.cvtColor(img_matt,cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(img_gray,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, color=(0, 0, 0), thickness=2)
    cv2.imshow('contours',img)
    


while True:
    ret,frame = cap.read()
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    detect_pic(frame,gray)

    
    key = cv2.waitKey(10)
    if key == 27:
        break
    
