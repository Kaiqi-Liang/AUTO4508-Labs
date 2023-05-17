#!/usr/bin/env python
from eye import *
from keras.models import load_model
import time
import numpy as np

from generate_data import Image_Processing

#Change model name here
Model_Name="PilotNet"

def PilotNet_Drive(model):
    img=CAMGet()
    LCDImageStart(0, 0, 320, 240)
    LCDImage(img)
    img=Image_Processing(img)
    display_img=img.ctypes.data_as(ctypes.POINTER(ctypes.c_byte))
    LCDImageStart(0, 0, 200, 66)
    LCDImage(display_img)
    model_input = np.expand_dims(img, axis=0)
    speed=int(np.round(model.predict(model_input)[0]))
    steering_angle=int(np.round(model.predict(model_input)[1]))
    LCDSetPrintf(19, 30, "Steering angle:= %d    ", steering_angle)
    LCDSetPrintf(19, 60, "Speed:= %d    ", speed)
    VWSetSpeed(speed,steering_angle)

def main():
    CAMInit(QVGA)
    done=0

    while True:
        LCDMenu("PilotNet Drive", " ", "", "END")
        k=KEYGet()
        if k == KEY1:
            LCDMenu("Auto Control", " ", " ", "Exit")
            LCDSetPrintf(18, 0, "PilotNet Drive")
            PilotNet_model=load_model(Model_Name)
            Iterations = 0
            Start_FPS = time.time()
            while True:
                PilotNet_Drive(PilotNet_model)
                Iterations += 1
                k=KEYRead()
                if k == KEY4:
                    End_FPS = time.time()
                    FPS = float(Iterations/(End_FPS - Start_FPS))
                    LCDSetPrintf(18, 30, 'Frames/Second : %.2f' %(FPS))
                    break
        elif k == KEY4:
            done = 1
        VWSetSpeed(0,0) # stop
        if done: break

main()
