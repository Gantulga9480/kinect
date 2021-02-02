import numpy as np
import paho.mqtt.client as mqtt
import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from shutil import move
from parameters import *


class PahoMqtt:

    def __init__(self, broker, info, port=1883,
                 raw_msg=False):

        self.__broker = broker
        self.__port = port
        self.info = info
        self.__client = mqtt.Client(f"{info} control")
        if not raw_msg:
            self.__client.on_message = self.__on_message
        else:
            self.__client.on_message = self.__on_message_raw
        self.__client.on_connect = self.__on_connect
        self.__client.on_publish = self.__on_publish
        self.__client.on_disconnect = self.__on_disconnect
        self.__client.wait_for_publish = self.__wait_for_publish
        self.__client.connect(self.__broker, self.__port)

        self.bridge = CvBridge()
        rospy.init_node(f'{self.info}_node')
        rospy.Subscriber('camera/rgb/image_color', Image, self.rgb_callback)
        rospy.Subscriber('camera/depth/image', Image, self.depth_callback)

        self.is_started = False

        self.is_running = True
        self.is_streaming = False
        self.is_idle = True
        self.is_playing = False

        self.frame_index = 0
        self.label_start = list()
        self.label_end = list()

        self.rgb_frame = None
        self.depth_frame = None

    def __on_connect(self, client, userdata, level, buf):
        self.publish(topic='kinect', msg=f'{self.info} connected')
        self.subscribe(topic='kinect')
        print(f"{self.info} connected")

    def __on_message(self, client, userdata, message):
        msg = message.payload.decode("utf-8", "ignore")
        msgs = msg.split("-")
        print(msgs)
        if msgs[0] == START:
            if self.is_started:
                pass
            else:
                self.reset()
                self.create_writer()
                self.is_started = True
                self.path = msgs[1]
            self.is_streaming = True
            self.is_idle = False
            if self.is_playing:
                cv2.destroyAllWindows()
            self.is_playing = False
        elif msgs[0] == STOP:
            self.is_streaming = False
            self.is_idle = True
            if self.is_playing:
                cv2.destroyAllWindows()
            self.is_playing = False
        elif msgs[0] == ACTIVITIE_START:
            self.label_start.append([f'{msgs[1]} start', self.frame_index])
        elif msgs[0] == ACTIVITIE_STOP:
            self.label_end.append([f'{msgs[1]} end', self.frame_index])
        elif msgs[0] == SAVE:
            self.save()
            self.reset()
        elif msgs[0] == RESET:
            self.reset()
        elif msgs[0] == PLAY:
            self.is_streaming = False
            self.is_playing = True
            self.is_idle = False
        elif msgs[0] == QUIT:
            self.is_running = False

    def reset(self):
        print('[INFO] RESET ...')
        self.is_streaming = False
        self.is_started = False
        self.is_idle = True
        if self.is_playing:
            cv2.destroyAllWindows()
        self.is_playing = False
        self.label_end.clear()
        self.label_start.clear()
        self.frame_index = 0
        try:
            del(self.rgb_out)
            del(self.depth_out)
        except Exception:
            pass
        print('[INFO RESET DONE]')

    def save(self):
        print('[INFO] SAVING ...')
        self.rgb_out.release()
        self.depth_out.release()
        os.makedirs(f'{self.path}')
        move(f'cache/{self.info}_rgb.avi', self.path)
        move(f'cache/{self.info}_depth.avi', self.path)
        print('[INFO] SAVING DONE')

    def rgb_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg).astype(np.uint8)
        self.rgb_frame = img
        if self.is_streaming:
            self.rgb_out.write(img)

    def depth_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB).astype(np.uint8)
        self.depth_frame = img
        if self.is_streaming:
            self.depth_out.write(img)

    def create_writer(self):
        self.rgb_out = cv2.VideoWriter(f"cache/{self.info}_rgb.avi",
                                       cv2.VideoWriter_fourcc(*'DIVX'),
                                       FPS, SIZE)
        self.depth_out = cv2.VideoWriter(f"cache/{self.info}_depth.avi",
                                         cv2.VideoWriter_fourcc(*'DIVX'),
                                         FPS, SIZE)

    def __on_message_raw(self, client, userdata, message):
        pass

    def __on_publish(self, client, userdata, result):
        print('[INFO] status published')

    def __on_disconnect(self, client, userdata, rc):
        print(f"[INFO] {self.info} disconnected")

    def __wait_for_publish(self):
        print('[INFO] waiting to publish status')

    def disconnect(self):
        self.__client.disconnect()

    def publish(self, topic, msg, qos=0):
        self.__client.publish(topic, payload=msg, qos=qos)

    def subscribe(self, topic, qos=0):
        self.__client.subscribe(topic, qos=qos)

    def loop_start(self):
        self.__client.loop_start()
