import cv2
import numpy as np
import tkinter as tk
import tkinter.filedialog



def init_prog():
    root = tk.Tk()
    root.withdraw()
    
file_name=tkinter.filedialog.askopenfilename(title="title")
img=cv2.imread(file_name,cv2.IMREAD_COLOR)


def color_judge(value):
    color_wheel=['red','orange','yellow-orange',
                 'yellow','yellow-grren','green',
                 'blue-green','green-blue','blue',
                 'blue-violet','violet','red-violet','red']
    index_value=int(round(value/180*(len(color_wheel)-1),0))
    return color_wheel[index_value]

def click_pos(event, x, y, flags, params):
    if event == cv2.EVENT_MOUSEMOVE:
        img2 = np.copy(img)
        cv2.circle(img2,center=(x,y),radius=5,color=255,thickness=-1)
        pos_str='(x,y)=('+str(x)+','+str(y)+')'
        imghsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        color=color_judge(imghsv[y,x,0])
        print(color)
        cv2.putText(img2,color,(x+10, y+10),cv2.FONT_HERSHEY_PLAIN,1,255,1,cv2.LINE_AA)
        cv2.imshow('window', img2)

def main():
    init_prog()
        
cv2.imshow('window', img)
cv2.setMouseCallback('window', click_pos)
cv2.waitKey(0)
cv2.destroyAllWindows()