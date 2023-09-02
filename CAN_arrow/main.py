# system
import sys
import threading
import time

#pygame
from pygame.locals import *
import pygame

#usr lib
from UsbCan import UsbCan
from UseMessage import UseMessage

#pygame init
pygame.init()
screen = pygame.display.set_mode((100,100))
pygame.display.set_caption("key board event")

#can init
Ucan = UsbCan()
Ucan.open()

msg_class = UseMessage(3) #id=1
msg_data = [1] * 8

def key_monitor():
    while True:
        screen.fill((0,0,0))
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_DOWN:
                    print("down")
                    msg_data[0] = 0
                elif event.key == K_UP:
                    print("up")
                    msg_data[0] = 2
                elif event.key == K_LEFT:
                    print("left")
                    msg_data[1] = 0
                elif event.key == K_RIGHT:
                    print("right")
                    msg_data[1] = 2
                elif event.key  == K_ESCAPE:
                    pygame.quit()
                    Ucan.close()
                    sys.exit()
            if event.type == KEYUP:
                msg_data[0] =  1
                msg_data[1] =  1
            pygame.display.update()
            
            
def can_trans():
    while 1:
        print(msg_data)
        tx_msg = msg_class.update_message(msg_data)
        Ucan.send(tx_msg)
        time.sleep(0.001)   
        
        
#thread
try:
    th1 = threading.Thread(target=key_monitor)
    th2 = threading.Thread(target=can_trans)
    th1.start()
    th2.start()
except KeyboardInterrupt:
    th1.join()
    th2.join()