from module.UsbCan import UsbCan
from module.UseMessage import UseMessage

import time

def main():
    Ucan = UsbCan(0)
    canmsg = UseMessage(0x200)
    msg = [0] * 8
    
    Ucan.open()
    
    try:
        while 1:
            msg[0] = 3
            msg[1] = 255
            msg[2] = 3
            msg[3] = 255
            msg[4] = 0
            msg[5] = 0
            msg[6] = 0
            msg[7] = 0
            msg_1 = canmsg.update_message(msg)
            Ucan.send(msg_1)
            time.sleep(0.001)
    except KeyboardInterrupt:
        Ucan.close()
        

if __name__ == "__main__":
    main()