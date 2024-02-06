#!/usr/bin/env python3.6

import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import String
import time
from subprocess import call


# Monitor /battery topic - if voltage is 14V or below, publish battery fault on /status

threshold_voltage = 14.0

def callback_battery(data_battery):
    if data_battery.data <= threshold_voltage:
        pub = rospy.Publisher('fault', String, queue_size=10)
        pub.publish('FAULT-BATTERY')
    else:
        pub = rospy.Publisher('fault', String, queue_size=10)
        pub.publish('None')
        pass
    time.sleep(1)
def battery_monitor():
    rospy.init_node('battery_monitor', anonymous=True)
    rospy.Subscriber("battery", Float32, callback_battery)
    rospy.spin()

if __name__ == '__main__':
    battery_monitor()
