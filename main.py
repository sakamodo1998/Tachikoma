import serial
import micropyGPS
import threading
import time
import gpiozero
import picamera
import picamera.array
import cv2 as cv
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
##------------------------------------------------------------------  get face
def getface():
    with picamera.PiCamera() as camera:
        # カメラの画像をリアルタイムで取得するための処理
        with picamera.array.PiRGBArray(camera) as stream:
            # 解像度の設定
            camera.resolution = (512, 384)

            while True:
                # カメラから映像を取得する（OpenCVへ渡すために、各ピクセルの色の並びをBGRの順番にする）
                camera.capture(stream, 'bgr', use_video_port=True)
                # 顔検出の処理効率化のために、写真の情報量を落とす（モノクロにする）
                grayimg = cv.cvtColor(stream.array, cv.COLOR_BGR2GRAY)

                # 顔検出のための学習元データを読み込む
                face_cascade = cv.CascadeClassifier('haarcascades/haarcascade_frontalface_default.xml')
                # 顔検出を行う
                facerect = face_cascade.detectMultiScale(grayimg, scaleFactor=1.2, minNeighbors=2, minSize=(10, 10))
                
                # 顔が検出された場合
                if len(facerect) > 0:
                    # 検出した場所すべてに赤色で枠を描画する
                    for rect in facerect:
                        cv.rectangle(stream.array, tuple(rect[0:2]), tuple(rect[0:2]+rect[2:4]), (0, 0, 255), thickness=3)

                # 結果の画像を表示する
                cv.imshow('camera', stream.array)

                # カメラから読み込んだ映像を破棄する
                stream.seek(0)
                stream.truncate()
                cv.waitKey(1)
                # 何かキーが押されたかどうかを検出する（検出のため、1ミリ秒待つ）
                #if cv.waitKey(1) > 0:
                #    break

            # 表示したウィンドウを閉じる
            cv.destroyAllWindows()              
facethread=threading.Thread(target=getface,args=())
facethread.daemon = True
facethread.start()# start thread    
##----------------------------------------------------------------------others
startled=gpiozero.LED(17)
##------------------------------------------------------------------wait for start
while(position[0]!=0.0):
    position=getgps()
    print("starting,please wait")
##----------------------------------------------------------------------main
while True:
    if step=="gps":
        startled.on()

        
        print("gps location =",getgps())
        print("Distance Data=",getDistance())
    if step=="face":
        print("creat the face program!!!!!!!!!!!")
