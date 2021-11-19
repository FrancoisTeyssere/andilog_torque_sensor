#!/usr/bin/env python

#Andilog WLC Torque Sensor Driver

import sys
import rospy
import bluetooth
import time
import re
import codecs
from math import fabs
from geometry_msgs.msg import WrenchStamped

class WLCDriver():
    def __init__(self, mac_address):
        self.address = mac_address

        self.start_measure_string = '0201FD03'
        self.modify_frequency_string = '0203F000FA03' #250Hz
        #self.modify_frequency_string = '0203F0006403' #100Hz
        self.stop_measure_string = '0201F703'
        self.read_sensor_capacity_string = '0201D403'
        self.read_sensor_unit_string = '0201D503'
        self.tare_string = '0202F50303' #tare with button not authorized
        #self.tare_string = '0202F51303' #tare with button authorized
        self.button_press_string = '0201f203'

        self.rotating_buffer = [0,0,0,0,0]
        self.buffer_size = 3
        self.indice = 0

        self.numeric_const_pattern = '[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )(?: [Ee] [+-]? \d+ ) ?'
        self.rx = re.compile(self.numeric_const_pattern, re.VERBOSE)

        self.wrench_msg = WrenchStamped()
        self.wrench_pub = rospy.Publisher("wlc_wrench", WrenchStamped, queue_size=1)

        self.socket = self.connect()



    def connect(self):
        while(True):
            try:
                s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                s.connect((self.address, 1))
                break
            except bluetooth.btcommon.BluetoothError as error:
                s.close()
                print("Could not connect: ", error, "; Retrying in 2s...")
                time.sleep(2)
        rospy.loginfo("WLC Torque Sensor Connected")
        return s

    def send_message(self, msg_string):
        self.socket.send(bytes.fromhex(msg_string))

    def initialize_sensor(self):
        self.send_message(self.stop_measure_string)
        data = self.socket.recv(1024)
        self.send_message(self.modify_frequency_string)
        data = self.socket.recv(1024)
        self.send_message(self.read_sensor_capacity_string)
        data = self.socket.recv(1024)
        self.send_message(self.read_sensor_unit_string)
        data = self.socket.recv(1024)
        self.send_message(self.tare_string)
        data = self.socket.recv(1024)
        self.send_message(self.start_measure_string)
        data = self.socket.recv(1024)
        rospy.loginfo("WLC Torque Sensor Initialized")

    def buffer_mean(self):
        mean = 0
        for i in (0,self.buffer_size-1):
            mean = mean + self.rotating_buffer[i]
        return mean/self.buffer_size

    def read_torque_value_and_publish(self, raw_data):
        if(raw_data.hex() != self.button_press_string): #ignore button press
            #Extract Float from bytes
            #find float in String
            float_string = codecs.decode(raw_data, 'UTF-8', 'backslashreplace').split('\r')
            if(len(float_string) == 2 and "." in float_string[0] and "Nm" in float_string[0]): #check if we have the whole decimal number, and only one in the message
                extracted_float = self.rx.findall(float_string[0]) 
                #print(raw_data, extracted_float)
                value = float(extracted_float[0])
                if(value < 10):
                    #see if value is close to last means value to remove outliers
                    if(fabs(self.buffer_mean() - value) < 2.5):
                        self.wrench_msg.wrench.torque.x = value
                        self.wrench_msg.header.stamp = rospy.Time.now()
                        self.wrench_pub.publish(self.wrench_msg)
                    self.rotating_buffer[self.indice] = value
                    self.indice = (self.indice + 1)%self.buffer_size


if __name__ == "__main__":

    rospy.init_node("wlc_driver")
    rate = rospy.Rate(500)
    driver = WLCDriver(rospy.get_param('/wlc_driver/mac_address'))

    #Initialize and start measurement
    driver.initialize_sensor()

    while not rospy.is_shutdown():
        data = driver.socket.recv(1024)
        driver.read_torque_value_and_publish(data)
        rate.sleep()

    #close socket at shutdown
    driver.socket.close()