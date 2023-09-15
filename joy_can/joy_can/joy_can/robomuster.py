from module.UsbCan import UsbCan
from module.UseMessage import UseMessage

import time

def main():
    Ucan = UsbCan(0)
    canmsg = UseMessage(0x030)
    msg = [0] * 5
    
    Ucan.open()
    
    try:
        while 1:
            msg[0] = 1
            msg[1] = 0
            msg[2] = 0
            msg[3] = 0
            msg[4] = 0
            msg_1 = canmsg.update_message(msg)
            Ucan.send(msg_1)
            time.sleep(0.001)
    except KeyboardInterrupt:
        Ucan.close()
        
main()