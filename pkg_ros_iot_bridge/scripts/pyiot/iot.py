"""This will be imported by node_action_server_iot_bridge for tasks"""
import time
import requests

import paho.mqtt.client as mqtt #import the client1

class print_colour:
    """Used to change colour of text in console"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):
    """IOT function callback"""
    msg = str(message.payload.decode("utf-8"))
    print("message received ", msg)
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)

def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url, arg_broker_port, 
                                arg_mqtt_topic, arg_mqtt_qos):
    """MQTT sub thread start"""
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1

# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
    """MQTT Publish"""
    try:   
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1) # wait

        mqtt_client.loop_stop() #stop the loop
        return 0
    except:
        return -1
# -----------------  HTTP PUB -------------------
def http_publish(arg_http_message):
    """HTTP Publish"""
    msg = arg_http_message
    print("(HTTP)Publishing message", msg)

    p1dict = eval(msg)
    x_res = p1dict['turtle_x']
    y_res = p1dict['turtle_y']
    theta = p1dict['turtle_theta']
    parameters1 = {"id":"Sheet1", "turtle_x":x_res,
                   "turtle_y":y_res, "turtle_theta":theta}
    url1 = "https://script.google.com/macros/s/AKfycbzHbZ229Ab0TA91vdOiUQwJuc_BtBjTHg6_qK7xtLwey3iT6O7P/exec"
    #parameters2 = {"id":"task1", "team_id":"VB_2113", "unique_id":"UaHaYix",
                   #"turtle_x":x_res, "turtle_y":y_res, "turtle_theta":theta}
    #url2 = "https://script.google.com/macros/s/AKfycbw850dk4moVgebU2GGe0PUQUvvg8jTpSjBQCawJt3_13vgujLk/exec"
    try:
        ret1 = requests.get(url1, params=parameters1)
        #ret2 = requests.get(url2, params=parameters2)
        ret = ret1.content
        return ret
    except:
        return -1
