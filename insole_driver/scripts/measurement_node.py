#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import logging
from ros_messages.msg import InsoleRawData
from ros_messages.msg import InsolePressure
from ros_messages.msg import InsoleTemperature
from ros_messages.msg import InsoleAcceleration

logging.basicConfig()
logging.getLogger('pygatt').setLevel(logging.DEBUG)

class Measurement(object):

    def __init__(self, rawBinary=None, temperature=None, accelerationX=None,
                 accelerationY=None, accelerationZ=None):
        test = rawBinary

    def publishTranslatedBinary(self, data):

        self.deviceName = data.deviceName
        self.pressPub = rospy.Publisher(self.deviceName+'/insole_pressure', InsolePressure, queue_size=1)
        self.tempPub = rospy.Publisher(self.deviceName+'/insole_temperature', InsoleTemperature, queue_size=1)
        self.accPub = rospy.Publisher(self.deviceName+'/insole_acceleration', InsoleAcceleration, queue_size=1)
        self.rawBinary = data.rawData

        try:

            self.pressureX = int(self.rawBinary[32:36], 16)
            self.pressureY = int(self.rawBinary[36:40], 16)
            self.pressureZ = int(self.rawBinary[40:44], 16)
            self.pressureTotal = self.pressureX+self.pressureY+self.pressureZ

            self.insolePressureMsg = InsolePressure(deviceName=self.deviceName,
                                           area1=self.pressureX,
                                           area2=self.pressureY,
                                           area3=self.pressureZ,
                                           areaTotal=self.pressureTotal)

            self.pressPub.publish(self.insolePressureMsg)

            self.accX = int(self.rawBinary[44:46], 16)
            self.accY = int(self.rawBinary[46:48], 16)
            self.accZ = int(self.rawBinary[48:50], 16)

            self.insoleAccelerationMsg = InsoleAcceleration(deviceName=self.deviceName, x=self.accX, y=self.accY,
                                                            z=self.accZ, w=self.accX+self.accY+self.accZ)

            self.accPub.publish(self.insoleAccelerationMsg)

            self.temp = int(self.rawBinary[4:8], 16)

            self.insoleTemperatureMsg = InsoleTemperature(deviceName=self.deviceName,
                                                 temperature=self.temp)

            self.tempPub.publish(self.insoleTemperatureMsg)

        except:
            pass


    def callback(self, data):
        self.publishTranslatedBinary(data)

if __name__ == '__main__':
    measurement = Measurement()

    rospy.Subscriber('/insole_rawdata', InsoleRawData, measurement.callback)

# Verschiedene Publisher einbauen
    rospy.init_node('Insole_Measurement', anonymous=False)

    rospy.spin()
