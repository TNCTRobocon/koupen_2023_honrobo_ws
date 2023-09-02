import cv2
import numpy as np
import tkinter as tk
import tkinter.filedialog
import os

pic_path = tkinter.filedialog.askopenfilename(title="title")
pic_name ,ext = os.path.splitext(os.path.basename(pic_path))
output_pic_name = str(pic_name) + "_out.jpg"
gray_pic_name = str(pic_name) + "_gray.jpg"

img = cv2.imread(pic_path,cv2.IMREAD_UNCHANGED)


def init_pic():
    height, width = img.shape[:2]
    
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV_FULL)
    h = hsv[:,:,0]
    s = hsv[:,:,1]
    v = hsv[:,:,2]
    
    img_matt = np.zeros((height,width,3),np.uint8)
    img_matt[(h < 120) & (h > 60) & (s > 50)& (v > 200)] = 255
    
    cv2.imwrite(gray_pic_name,np.array(img_matt))
    img_gray = cv2.imread(gray_pic_name,cv2.IMREAD_GRAYSCALE)
    M = cv2.moments(img_gray,False)
    contours, hierarchy = cv2.findContours(img_gray,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #x,y= int(M["m10"]/M["m00"]) , int(M["m01"]/M["m00"])
    #cv2.circle(img, (x,y), 20, 100, 2, 4)
    cv2.drawContours(img, contours, -1, color=(0, 0, 0), thickness=5)
    
    cv2.imwrite(output_pic_name,np.array(img))
    
    
    #cv2.imshow('window', )
    #print(x,y)
    if not __debug__:
        print("input_path:{}".format(pic_name))
        print("output_path:{}".format(output_pic_name))
    
    
#main
def init():
    init_pic()
    
cv2.imshow('window', img)
