import os
import can

class UsbCan:
    """constructor"""
    def __init__(self) :
        self.__state = False
        return
    
    """destructor"""
    def __del__(self):
        if self.__state == True:
            os.system('sudo ifconfig can0 down')
        return
    
    """open can port"""
    def open(self):
        self.__state = True
        try:
            os.system('sudo ip link set can0 type can bitrate 500000')
            os.system('sudo ifconfig can0 up')
            os.system('sudo ifconfig can0 txqueuelen 10000') #バッファオーバーフロー対策 
            self.__can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')
        except OSError:
            print('OS Error')
        return
    
    """close can port"""
    def close(self):
        self.__state = False
        os.system('sudo ifconfig can0 down')
        
        return
    
    """send data"""
    def send(self,msg):
        if self.__state == True:
            self.__can0.send(msg)
        return
    