import serial
import micropyGPS
import threading
import time
##--------------------------------------------------------------set paramate
TargetPosition=[] #set Target gps position [laitude,longitude, altitude]
GPStimeout=10   # gps timeout (secend)
step="gps"   # start with gps guide or face guide

##------------------------------------------------------------defut paramate
position=[0,0,0]#lattitude,longitude,altitude
#buff=""
#buffData=""

##-----------------------------------------------------------------------GPS 
gps = micropyGPS.MicropyGPS(9,'dd')#timezone:9 , timeformat:dd
def rungps():
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline()    
    while True:
        sentence = s.readline().decode('utf-8') #GPS data 2 str
        if sentence != "": # makesure info is in the sentence
            if sentence[0] != '$': #real gps data start with '$'
                continue
            for x in sentence: #send msg 2 micropyGPS
                gps.update(x)
gpsthread=threading.Thread(target=rungps,args=())
gpsthread.daemon = True
gpsthread.start()# start thread
def getgps():
    position[0]=float(gps.latitude[0])
    position[1]=float(gps.longitude[0])
    position[2]=float(gps.altitude)
    return position
##-----------------------------------------------------------distance Sensor
serDistance=serial.Serial('/dev/ttyACM0',115200)
def getDistance():
    buff=""
    buffData=""
    data=[0.0,0.0,0.0,0.0,0.0,0.0]
    port=[0,1,2,3,4,5]
    serDistance.readline()
    for x in port:
        #print(serDistance.readline())
        while True:
            #print("geting")
            buff = serDistance.read().decode()
            #print(buff)
            if(buff != ','and buff!="$" and buff!="/r" and buff!="/n"):
                #print("getdata")
                buffData += str(buff)
                #print(buffData)
            elif(buff==','):
                #print("----------------------changeidex")
                data[x] = float(buffData)
                buffData = ""
                break
            elif(buff=="$"):
                #print("---------------------stop reading")
                data[x]= float(buffData)
                serDistance.read()
                serDistance.read()
                break
    return data
    #print("%f,%f,%f,%f,%f,%f" %
    #    (data[0], data[1], data[2], data[3], data[4], data[5]))
    #data=[0.0,0.0,0.0,0.0,0.0,0.0]
##----------------------------------------------------------------------main

while True:
    if step=="gps":
        print("gps location =",getgps())
        print("Distance Data=",getDistance())
    if step=="face":
        print("creat the face program!!!!!!!!!!!")