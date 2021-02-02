import paho.mqtt.client as mqtt
import argparse

broker = '192.168.0.100'
port = 1883

parser = argparse.ArgumentParser()
parser.add_argument('-m', '--message')
args = parser.parse_args()


def on_pub(client, usrdata, result):
    print('command sent')


client = mqtt.Client()
client.on_publish = on_pub
client.connect(broker, port)
client.publish('kinect1', payload=args.message, qos=0)
client.loop_start()
