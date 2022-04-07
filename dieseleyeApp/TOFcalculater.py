from ast import literal_eval
from binascii import crc32
from turtle import delay
import time
import serial
crcCheck = False
openCheck = False
class TOFSerial(object):
    def setupSerial(self,port : str, baudrate: int): #call once
        self.ser = serial.Serial(port, baudrate)
    
    def startSerial(self , buffer : int): #call in interverl
        global dec
        self.s = self.ser.readline(buffer).decode('ascii')
        #time.sleep(1)
        data = str(self.s)
        if self.ser.is_open:
            openCheck = True 
        if (data.startswith('c1:') and data.find(',.') != -1 ):
            # print('can be parsed')
            split = data.split(',')
            print(split)
            reducedC1 = split[0].replace('c1:', '')
            reducedC2 = split[1].replace('c2:', '')
            reducedT1 = split[2].replace('t1:', '')
            reducedT2 = split[3].replace('t2:','')
            reducedclkcont = split[4].replace('clkc:','')
            
            crcCheck = True
        else:
            crcCheck = False
        if crcCheck == True:
            # print(reducedC1)
            # print(reducedC2)
            # print(reducedT1)

            cal1 = float(reducedC1)
            cal2 = float(reducedC2)
            time1 = float(reducedT1)
            time2 = float(reducedT2)
            clkcount = float(reducedclkcont)
            
            if cal1 and cal2 and time1 and time2 and clkcount != 0:
                calcount = (cal2-cal1)/(10-1)
                normlsb = (1.25e-7)/calcount
                tof = (time1*normlsb)+(clkcount*1.25e-7)-(time2*normlsb)
                TOF = str(tof)

                # print(TOF)
                dec = "{:.8f}".format(float(TOF)*1000000)#*1000000000
                print('time of flight is' , dec +" us")
                return(dec)

    def stopSerial(self):
        if openCheck == True:
            self.ser.close()
    
if __name__ == '__main__':
    tof = TOFSerial()
    tof.setupSerial("com33",9600)
    while 1: 
        tof.startSerial(70)
    