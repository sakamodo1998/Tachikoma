# Tachikoma

## HARDWEAR
raspberry pi 4B+ 4GB (raspberry pi os)  
arduino uno  
CAMERA: PiCamera  
LCD: ACM1602N1-FLW-FBW  
GPS: GYSFDMAXB  
DISTANCE SENSOR: GP2Y0A21YK0F  

## CONNECTION
distance sensor --> arduino(A0~A5) --usb--> raspberry pi  
GPS --> raspberry pi(RTX,DTX0)  

## SETUP

1. write "distancesensor.ino" into arduino uno  
2. run the following code to setup opencv  
```
sudo pip3 install opencv-python
sudo apt-get install libatlas-base-dev
sudo apt-get install libjasper-dev
sudo apt-get install libqt4-test
```
