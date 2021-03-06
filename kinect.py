from paho_mqtt import PahoMqtt as Host, cv2
import numpy as np
import time
from parameters import *

kinect = Host(BROKER, NODE)
kinect.loop_start()


while kinect.is_running:
    if kinect.is_streaming:
        print(f'[INFO] {kinect.info} is streaming')
        time.sleep(1)
    elif kinect.is_idle:
        print(f'[INFO] {kinect.info} is in idle')
        time.sleep(1)
    elif kinect.is_playing:
        print(f'[INFO] {kinect.info} is playing')
        if np.any(kinect.rgb_frame):
            cv2.imshow(f'{kinect.info} color', kinect.rgb_frame)
            cv2.imshow(f'{kinect.info} depth', kinect.depth_frame)
            cv2.waitKey(33)
