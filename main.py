import serial
import micropyGPS
import threading
import time
from time import sleep 
import numpy
import gpiozero
import picamera
import picamera.array
import cv2 as cv
import smbus		
import math
##----------------------------------------------------------------初期設定
TargetPosition=[0,0,0] #お客さんの位置（緯度,経度、海抜（多分使わん））
GPStimeout=10   #gpsのタイムアウトの時間の設定（秒
step="gps"   #"gps"でgps制御、"face"で顔位置制御から始まる
SETresolution=[512,512]#解像度
delaytime=0.1#周波数=10Hz
stopsize=500#十分近づいたと判断する顔の対角線の長さ
kp=0.05
kd=0.05
there_is_a_wall=50#壁があると判断する距離[cm]
long_twolegs=20#片側二足の距離
##------------------------------------------------------------処理のために格納する変数の宣言
faceposition=[0,0,0]
position=[0,0,0]
mistery=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
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
    data=[0.0,0.0,0.0,0.0,0.0,0.0]
    serDistance.readline()
    linedata=serDistance.readline().decode("utf-8")
    print(linedata)
    while True:
        if(linedata[0]=="#"):
            i=1
            port=0
            while(linedata[i]!="$"):
                if(linedata[i]!="@"):
                    print("-------",linedata[i])
                    buff+=linedata[i]
                    print(buff)
                elif(linedata[i]=="@"):
                    data[port]=float(buff)
                    buff=""
                    print(data,port)
                    port+=1
                i+=1
            data[5]=float(buff)
            buff=""
            #print(data)
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
##----------------------------------------------------------------デジタルコンパス
X_axis_H    = 0x01
Z_axis_H    = 0x05              
Y_axis_H    = 0x03             
declination = -0.00669        
pi          = 3.14159265359

bus = smbus.SMBus(1) 	
Device_Address = 0x0d

def read_raw_data(addr):
    low = bus.read_byte_data(Device_Address, addr-1)
    high = bus.read_byte_data(Device_Address, addr)
    value = ((high << 8) | low)
    if value >= 0x8000:
        value = value - 0x10000
        return value
    else:
        return value
def getangle():
    i=0
    bus.write_byte_data(Device_Address, 0x0a, 0x81)
    bus.write_byte_data(Device_Address, 0x0b, 0x01)
    bus.write_byte_data(Device_Address, 0x09, 0x11)     
    #heading = None
    #[x,y,z] = [None,None,None]
    while i<20:
        status = bus.read_byte_data(Device_Address, 0x06)
        if status == 0x04:
            x = read_raw_data(X_axis_H)
            y = read_raw_data(Y_axis_H)
            z = read_raw_data(Z_axis_H)
            continue
        if status == 0x01:
            x = read_raw_data(X_axis_H)
            y = read_raw_data(Y_axis_H)
            z = read_raw_data(Z_axis_H)
            break
        else:
            sleep(0.01)
            i+=1
    if x is None or y is None:
        [x1, y1] = [x, y]
    else:
        c=[[1.0, 0.0, 0.0],
           [0.0, 1.0, 0.0],
           [0.0, 0.0, 1.0]]
        x1 = x * c[0][0] + y * c[0][1] + c[0][2]
        y1 = x * c[1][0] + y * c[1][1] + c[1][2]
    if x1 is None or y1 is None:
        heading = 0
    else:
        heading=math.degrees(math.atan2(y1,x1))
    if(heading < 0):
        heading = heading + 360.0 + math.degrees(declination)
    elif(heading > 360.0):
        heading = heading - 360.0
    if(heading>180):
        heading=heading-360
    return (heading)   
##----------------------------------------------------------------サーボモーター（方向

##----------------------------------------------------------------動力モータ（速度

##---------------------------------------------------------------回転
def roll(targetAngle,Kp,Kd,):
    print("----------------------rolling")
    ANGLE=getangle()
    old_E=0
    while (abs(ANGLE-targetAngle)>5):
            ##サーボモータを回転の角度をする !!
            ANGLE=getangle()
            E=targetAngle-ANGLE#目標ー現在
            DE=(E-old_E)/delaytime
            U=Kp*E-DE*Kd#制御値
            old_E=E
            ##制御値をモータドライバーに入力 !!
            time.sleep(delaytime)
    ##サーボモータの角度まっすぐ前方にする！！
    print("----------------------Done")
    
##----------------------------------------------------------------その他の初期化
startGPSled=gpiozero.LED(17)
startFACEled=gpiozero.LED(27)
##----------------------------------------------------------------gpsの起動は時間がかかるからここで待つ
while(position[0]!=0.0):
    position=getgps()
    print("starting,please wait")

################################################################################   MAIN

if step=="gps":
    startGPSled.on()

    nowlocation=getgps()
    angle=getangle()
    distancedata=getDistance()#distancedata[前、右前、右後、後、左後、左前]
    
    #----------------------------------------------頭をターゲットの方向に向く
    theta=90+math.degrees(math.atan2((TargetPosition[1]-nowlocation[1]),(TargetPosition[0]-nowlocation[0])))
    old_ea=0
    roll(angle-theta,kp,kd)
    #----------------------------------------------目標に進む
    old_ed=0
    old_ex=0
    while (abs(nowlocation[0]-TargetPosition[0])>=0.00004 and abs(nowlocation[1]-TargetPosition[1])>=0.00004):    
        distancedata=getDistance()
        #---------------------------------------------------------前方に障害物がある場合
        if(distancedata[0]<=there_is_a_wall):
            roll(getangle()+90,kp,kd)#多分左側の道に降ろされるから障害物がある場合右のほうがあいてる可能性が高い,角度座標系：逆時計-、時計回り+
            distancedata=getDistance()
            while(distancedata[4]<=there_is_a_wall):
                distancedata=getDistance()
                print("--->")
                if(distancedata[5]<=there_is_a_wall):#障害物の角度がなめらかに変化する場合はそれに沿って徐行する
                    wallangle=math.degrees(math.atan((distancedata[5]-distancedata[4])/long_twolegs))
                    # wallangleをサーボモータに書き込む（角度が逆の可能性が大きい !!
                    #一定のスピードで進む　！！
                elif(distancedata[5]>there_is_a_wall):#前足左側の障害物が急に消える場合は後ろの足も抜け出して、障害物を避けるプロセス終了
                    wallangle=0
                    # サーボの角度をゼロにしてまっすぐ進む
            nowlocation=getgps()
            theta=90+math.degrees(math.atan2((TargetPosition[1]-nowlocation[1]),(TargetPosition[0]-nowlocation[0])))
            angle=getangle()
            roll(theta,kp,kd)
            old_ed=0
            old_ex=0        
        #---------------------------------------------------------まっすぐ進める場合
        nowlocation=getgps()
        angle=getangle()
        distancedata=getDistance()
        #theta=90+math.degrees(math.atan2((TargetPosition[1]-nowlocation[1]),(TargetPosition[0]-nowlocation[0])))
        ed=theta-angle#目標値（画像の真ん中）ー現在の位置
        ded=(ed-old_ed)/delaytime
        ud=kp*ed-ded*kd#制御値
        old_ed=ed
        ###ここでサーボに角度を書き込む !!
        ###速度を書き込む
    #----------------------------------------------change
    step="gps" #目標との距離が5ｍぐらいまで縮まったら顔位置制御にはいる
    startGPSled.off()
if step=="face": #顔位置制御
    print("start face navi")
    startFACEled.on()
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
        ex=stopsize-faceposition[2]
        dex=(ex-old_ex)/delaytime
        ux=kp*ed-dex*kd#制御値
        old_ex=ex
        ###ここで動力モータの制御値を書き込む
        print("pid_servo=",ud)
        print("pid_speed=",ux)
        
        time.sleep(delaytime)
    print("arrived")
