import cv2
import numpy as np
import os
import time

save_period = 1.0
data_dir = "output"
num_samples = 100
prefix = "frame_"
suffix = ".png"

def collect_data():

    #create data directory for images
    if(not os.path.exists(data_dir)):
        os.mkdir(data_dir)

    #camera setup
    vid = cv2.VideoCapture(0)

    #set desired camera properties
    print("Setting up camera properties")
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # vid.set(cv2.CAP_PROP_AUTO_EXPOSURE,3)
    # vid.set(cv2.CAP_PROP_EXPOSURE, 40) 
    # vid.set(cv2.CAP_PROP_FPS,5)
    # print("Auto Exposure:",vid.get(cv2.CAP_PROP_AUTO_EXPOSURE))
    # print("Exposure:",vid.get(cv2.CAP_PROP_EXPOSURE))

    counter = 0
    last_time = time.time()
    while(counter < num_samples):        
        ret, frame = vid.read()
        cv2.imshow('frame', frame)
        if(cv2.waitKey(1) & 0xFF == ord('q')):
            exit(1)

        if(time.time() - last_time > save_period):
            last_time = time.time()
            counter +=1
            save_image(frame,counter)

    print("Sample collection complete")


def save_image(frame, frame_number):
    file_name = prefix + str(frame) + "." + suffix
    full_path = os.path.join(data_dir,file_name)
    cv2.imwrite(full_path,frame)


if(__name__ == '__main__'):
    collect_data()