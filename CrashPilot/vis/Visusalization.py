from vpython import *
from time import *
import numpy as np
import math
import serial
import convert_stl as c2stl

ad=serial.Serial('com16',115200)
sleep(1)
 
#scene.range=5
scene.background=color.white
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(-1,-1,-1)
 
#scene.width=1200
#scene.height=1080
 
myObj = c2stl.stl_to_triangles("NAUD2.STL")

while (True):
    try:
        while (ad.inWaiting()==0):
            pass
        dataPacket=ad.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(",")
        q0=float(splitPacket[0])
        q1=float(splitPacket[1])
        q2=float(splitPacket[2])
        q3=float(splitPacket[3])
 
        roll=(-math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)))+45
        pitch=(math.asin(2*(q0*q2-q3*q1)))
        yaw=(-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2)-90
 
        rate(50)
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)
 
        #frontArrow.axis=k
        #sideArrow.axis=cross(k,vrot)
        #upArrow.axis=vrot
        myObj.axis=k
        myObj.up=vrot
        #sideArrow.length=2
        #frontArrow.length=4
        #upArrow.length=1
    except:
        pass