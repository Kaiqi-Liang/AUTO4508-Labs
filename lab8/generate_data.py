from eye import *
import os
import time
import cv2
import numpy as np
import pygame
from pygame.locals import *

#frame rate
FRAME_RATE=20

#manual control
#TO DEFINE
# max_speed=
# min_speed=
speed=0
speed_increment = 20
max_angspeed = 30
# min_steer=
angspeed=0
# steer_increment = 
#image recording
Image_Count = 0
Record_Button = 0
Recording_Data = False
# Default image format: RGB 
#################################################################
Script_Path = os.path.dirname(os.path.abspath(__file__))
Image_Path = os.path.join(Script_Path, 'ImageDatasets')

def Image_Processing(img):
    img = np.array(img, dtype=np.uint8)
    img = np.reshape(img, (240, 320, 3))
    img = img[100:, :]
    img = cv2.resize(img, (200, 66))
    return img

def Manual_Control():
    global speed
    global angspeed
    global Record_Button
    global Image_Count
    global Recording_Data
    global Image_Path
    img=CAMGet()
    img=Image_Processing(img)
    display_img=img.flatten().ctypes.data_as(ctypes.POINTER(ctypes.c_byte))
    LCDImageStart(0, 0, 200, 66)
    LCDImage(display_img)
    keys = pygame.key.get_pressed()
    if keys[K_r] and (pygame.time.get_ticks() - Record_Button) > 200:
        Record_Button=pygame.time.get_ticks()
        Recording_Data = not Recording_Data
        if Recording_Data:
            print("Recording Started")
            if os.path.isdir(Image_Path):
                Image_Count = len(os.listdir(Image_Path))
            else:
                Image_Count = 0
                os.makedirs(Image_Path)
        else:
            print("Recording Stopped")

    #Manual control by keyboard or joystick
    if keys[K_UP]:
        speed += speed_increment
    elif keys[K_DOWN]:
        speed -= speed_increment
    elif keys[K_LEFT]:
        if angspeed < max_angspeed:
            angspeed += 1
    elif keys[K_RIGHT]:
        if angspeed > -max_angspeed:
            angspeed -= 1
    else:
        speed = 200
        angspeed = 0

    VWSetSpeed(speed, angspeed)
    if Recording_Data:
        cv2.imwrite(f'{Image_Path}/{Image_Count}_{speed}_{angspeed}.png', img)
        Image_Count += 1

if __name__ == '__main__':
    CAMInit(QVGA)
    done=0
    while True:
        LCDMenu("Manual Drive", " ", " ", "END")
        k=KEYGet()
        if k == KEY1:
            LCDMenu("Manual Control", " ", " ", "Exit")
            LCDSetPrintf(18, 0, "Manual Drive")
            pygame.init()
            pygame.font.init()
            pygame.display.set_mode((400, 100))
            pygame.display.set_caption("Manual Control Window")
            Iterations = 0
            Start_FPS = time.time()
            while True:
                pygame.time.Clock().tick(FRAME_RATE)
                pygame.event.pump()
                Manual_Control()
                Iterations += 1
                if KEYRead() == KEY4:
                    End_FPS = time.time()
                    FPS = float(Iterations/(End_FPS - Start_FPS))
                    LCDSetPrintf(18, 30, 'Frames/Second : %.2f' %(FPS))
                    break
        elif k == KEY4:
            done = 1
        VWSetSpeed(0,0) # stop
        if done: break
