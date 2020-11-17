import serial
import micropyGPS
import threading
import time
import numpy
import gpiozero
import picamera
import picamera.array
import cv2 as cv
##----------------------------------------------------------------初期設定
TargetPosition=[0,0,0] #お客さんの位置（緯度,経度、海抜（多分使わん））
GPStimeout=10   #gpsのタイムアウトの時間の設定（秒
step="gps"   #"gps"でgps制御、"face"で顔位置制御から始まる
SETresolution=[512,512]#解像度
delaytime=0.1#周波数=10Hz
stopsize=500#十分近づいたと判断する顔の対角線の長さ
kp=0.05
kd=0.05
##------------------------------------------------------------処理のために格納する変数の宣言
faceposition=[0,0,0]
position=[0,0,0]
##-----------------------------------------------------------------------GPS 
gps = micropyGPS.MicropyGPS(9,'dd')#timezone:9 , timeformat:dd
def rungps():
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline()    
    while True:
        sentence = s.readline().decode('utf-8') #シリアルで撮ったバイトがたのデータを文字列に変換
        if sentence != "": #データが格納されてないことによるインデックスエラを防ぐ、空でないことを確認
            if sentence[0] != '$': #開始の表示を捨てる
                continue
            for x in sentence: 
                gps.update(x)
gpsthread=threading.Thread(target=rungps,args=())
gpsthread.daemon = True
gpsthread.start()# スレッド開始
def getgps():
    position[0]=float(gps.latitude[0])
    position[1]=float(gps.longitude[0])
    position[2]=float(gps.altitude)
    return position

##-----------------------------------------------------------赤外線距離センサのスレッド
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
##------------------------------------------------------------------顔認識のスレッド
def getface():
    faceindex=[0,1,2]
    with picamera.PiCamera() as camera:
        # カメラの画像をリアルタイムで取得するための処理
        with picamera.array.PiRGBArray(camera) as stream:
            camera.resolution = SETresolution # 解像度の設定
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
                    print("-----")
                    for rect in facerect:
                        #print(rect)
                        cv.rectangle(stream.array, tuple(rect[0:2]), tuple(rect[0:2]+rect[2:4]), (0, 0, 255), thickness=3)
                        cv.line(stream.array,(256,0),(256,512),(255,0,0),1)
                        cv.circle(stream.array,(rect[0]+int(rect[2]/2),rect[1]+int(rect[3]/2)),3,(0,0,255),4)
                        faceposition[0]=rect[0]+rect[2]/2
                        faceposition[1]=rect[1]+rect[3]/2
                        faceposition[2]=numpy.sqrt(rect[2]*rect[2]+rect[3]*rect[3])
                # 結果の画像を表示する
                cv.imshow('camera', stream.array)
                # カメラから読み込んだ映像を破棄する
                stream.seek(0)
                stream.truncate()
                cv.waitKey(1)                
            cv.destroyAllWindows()              

facethread=threading.Thread(target=getface,args=())
facethread.daemon = True
facethread.start()    
##----------------------------------------------------------------サーボモーター（方向

##----------------------------------------------------------------動力モータ（速度

##----------------------------------------------------------------その他の初期化
startGPSled=gpiozero.LED(17)
startFACEled=gpiozero.LED(27)
##----------------------------------------------------------------gpsの起動は時間がかかるからここで待つ
while(position[0]!=0.0):
    position=getgps()
    print("starting,please wait")
##----------------------------------------------------------------------main
while True:
    if step=="gps":
        startFACEled.on()
        nowlocation=getgps()
        print("gps location =",nowlocation)
        print("Distance Data=",getDistance())
        if(abs(nowlocation[0]-TargetPosition[0])<=0.00004 and abs(nowlocation[1]-TargetPosition[1])<=0.00004):
            step="face" #目標との距離が5ｍぐらいまで縮まったら顔位置制御にはいる
        time.sleep(delaytime)

    if step=="face":
        print("start face navi")
        old_ed=0
        old_ex=0
        while(faceposition[2]<stopsize):
            print(faceposition)
            #---------------------------------------------------PID(PD)
            ed=256-faceposition[0]#目標値（画像の真ん中）ー現在の位置
            ded=(ed-old_ed)/delaytime
            ud=kp*ed-ded*kd#制御値
            old_ed=ed
            ###ここでサーボに角度を書き込む
            print(ed,ded,ud)

            ex=stopsize-faceposition[2]
            dex=(ex-old_ex)/delaytime
            ux=kp*ed-dex*kd#制御値
            old_ex=ex
            ###ここで動力モータの制御値を書き込む
            print(ex,dex,ux)
            
            time.sleep(delaytime)
        print("arrived")
        break
    break
