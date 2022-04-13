from ast import literal_eval
from binascii import crc32
from socket import timeout
from turtle import delay
import time
import serial

#String payload = ("c1:"+String(cal1)+","+"c2:"+String(cal2)+","+"t1:"+String(time1)+","+"t2:"+String(time2)+","+"clkc:"+String(clkcount)+",.");
class TOFSerial(object):
    crcCheck = False
    openCheck = False
    def setupSerial(self,port : str, baudrate: int): #call once
        self.ser = serial.Serial(port, baudrate,timeout=1)
    
    def startSerial(self , buffer : int): #call in interverl
        global dec
        self.s = self.ser.readline(buffer).decode('ascii')
        #time.sleep(1)
        data = str(self.s)
        value = data.replace(' E-','e-')
        if self.s.endswith('E+9'):
            print("garbage value")
        print(value)
        return(value)
        #print(data)
    
        

    def stopSerial(self):
        if openCheck == True:
            self.ser.close()
    
if __name__ == '__main__':
    tof = TOFSerial()
    tof.setupSerial("com33",9600)
    while 1: 
        tof.startSerial(30)
    