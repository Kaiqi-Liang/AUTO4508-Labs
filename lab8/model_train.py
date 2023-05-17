#!/usr/bin/env python
from keras.callbacks import EarlyStopping,ReduceLROnPlateau,TensorBoard
from keras.optimizers import adam_v2
from keras.models import Model
from keras.layers import Input, Conv2D, Dropout, Flatten, Dense, Lambda
import numpy as np
import datetime
from sklearn.model_selection import train_test_split
import cv2
import os
import sys
from os.path import join, exists, dirname, abspath

Images_All = []
Speeds_All = []
Steering_Angles_All = []

Script_Path = dirname(abspath(__file__))
Image_Path = join(Script_Path, 'ImageDatasets')
if exists(Image_Path):
    for File in os.listdir(Image_Path):
        Speed=int(File.split("_")[1])
        Steering_Angle=int(File.split("_")[2].split('.')[0])
        Image=cv2.imread(join(Image_Path, File))
        Images_All.append(Image)
        Speeds_All.append(Speed)
        Steering_Angles_All.append(Steering_Angle)
else:
    print('[Error!] Check Image Directory is Correct!')
    sys.exit()
Split_a = train_test_split(Images_All, Speeds_All, Steering_Angles_All, test_size=0.1, shuffle=True)
(Images, Image_Test, Speeds, Speed_Test, Steering_Angles, Steering_Angle_Test) = Split_a   
Split_b = train_test_split(Images, Speeds, Steering_Angles, test_size=0.2, shuffle=True)
(Image_Train, Image_Valid, Speed_Train, Speed_Valid, Steering_Angle_Train, Steering_Angle_Valid) = Split_b
X_Train=np.asarray(Image_Train)
X_Valid=np.asarray(Image_Valid)
X_Test=np.asarray(Image_Test)
Y_Train=[(np.asarray(Speed_Train)), (np.asarray(Steering_Angle_Train))]
Y_Valid=[(np.asarray(Speed_Valid)), (np.asarray(Steering_Angle_Valid))]
Y_Test=[(np.asarray(Speed_Test)), (np.asarray(Steering_Angle_Test))]

def NVIDIA_PilotNet():
    Input_Shape = Input(shape=(66, 200, 3))
    X = Input_Shape
    X = (Lambda(lambda x: (x/127.5 - 1.0), name='Normalisation'))(X)
    X = (Conv2D(24, (5, 5), strides=(2, 2), activation='elu', name='Conv_1'))(X)

    X = (Conv2D(36, (5, 5), strides=(2, 2), activation='elu', name='Conv_2'))(X)

    X = (Conv2D(48, (5, 5), strides=(2, 2), activation='elu', name='Conv_3'))(X)

    X = (Conv2D(64, (3, 3), strides=(1, 1), activation='elu', name='Conv_4'))(X)

    X = (Conv2D(64, (3, 3), strides=(1, 1), activation='elu', name='Conv_5'))(X)
    X = (Flatten())(X)
    X = (Dropout(0.2))(X)
    X = (Dense(100, activation='elu'))(X)
    X = (Dense(50, activation='elu'))(X)
    X = (Dense(10, activation='elu'))(X)
    Model_Speed = (Dense(1, name='Model_Speed'))(X)
    Model_Steering = (Dense(1, name='Model_Steering'))(X)
    model = Model(inputs = [Input_Shape], outputs = [Model_Speed, Model_Steering])
    model.compile(loss=['mse', 'mse'], optimizer=adam_v2.Adam(learning_rate=1e-3), metrics=['mse'])
    model.summary() 
    return model

Stopping = EarlyStopping(monitor='val_loss', mode='min', verbose=0, patience=20)
ReduceLR = ReduceLROnPlateau(monitor='val_loss', factor=0.2, patience=7, min_lr=0.000001, verbose=2)
TensorBoard_Stamp="tb_callback"
Tensorboard_Callback=TensorBoard(log_dir=join(Script_Path, TensorBoard_Stamp), histogram_freq=1)
Callbacks = [Stopping, ReduceLR, Tensorboard_Callback]
model=NVIDIA_PilotNet()
history = model.fit(X_Train, Y_Train, validation_data=(X_Valid, Y_Valid), epochs=100,verbose=2,callbacks=Callbacks)

print(f"Model Test Loss: {model.evaluate(X_Test, Y_Test, verbose=2)[0]:.4f}\n")
Model_Stamp="PilotNet"
model.save(join(Script_Path, Model_Stamp))