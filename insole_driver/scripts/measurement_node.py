#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import logging
import time
from ros_messages.msg import InsoleRawData
from ros_messages.msg import InsolePressure

logger = logging.getLogger('Measurement-Insole')


class Measurement(object):

    def __init__(self):

        self.init_logger()

        self.dataBuffer = []
        self.minVal = 0
        self.locked = False

    @staticmethod
    def init_logger():
        # create logger with 'spam_application'
        logger.setLevel(logging.DEBUG)
        # create file handler which logs even debug messages
        fh = logging.FileHandler('./log/event_time.log')
        fh.setLevel(logging.DEBUG)
        # create console handler with a higher log level
        ch = logging.StreamHandler()
        ch.setLevel(logging.ERROR)
        # create formatter and add it to the handlers
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        # add the handlers to the logger
        logger.addHandler(fh)
        logger.addHandler(ch)

    def convert_and_publish_binary(self, data):

        if data is not None:
            self.deviceName = data.deviceName
            self.rawBinary = data.rawData
            #self.pressPub = rospy.Publisher('/insole_pressure/'+self.deviceName, InsolePressure, queue_size=1)
            self.pressPub = rospy.Publisher('/insole_pressure', InsolePressure, queue_size=1)
            try:
                self.convert_raw_to_pressure()
                skipIteration = self.totalPressure < 10

                #  skip outlier
                if not skipIteration:
                    self.remove_dynamic_offset()
                    self.create_pressure_msg()
                    self.publish_pressure_msg()

            except Exception as e:
                logger.exception('Got exception on main handler: '+e.message)

    def publish_pressure_msg(self):

        try:
            self.pressPub.publish(self.insolePressureMsg)

        except Exception:
            logger.exception('Couldnt publish measurement.')

    def create_pressure_msg(self):
        self.insolePressureMsg = InsolePressure(deviceName=self.deviceName,
                                                area1=self.pressureX,
                                                area2=self.pressureY,
                                                area3=self.pressureZ,
                                                areaTotal=self.totalPressure)

    def convert_raw_to_pressure(self):
        self.pressureX = int(self.rawBinary[32:36], 16)
        self.pressureY = int(self.rawBinary[36:40], 16)
        self.pressureZ = int(self.rawBinary[40:44], 16)
        self.totalPressure = self.pressureX + self.pressureY + self.pressureZ

    def remove_dynamic_offset(self):
        if self.totalPressure > 40:
            self.dataBuffer.append(self.totalPressure)
            if (self.totalPressure - self.minVal) < 0:  # in case that min value is greater then measurement
                self.minVal = 0
            if len(self.dataBuffer) > 200:  # determine lower limit over 200 iterations
                self.minVal = min(self.dataBuffer) if min(self.dataBuffer) > self.minVal else self.minVal
                #print(self.dataBuffer)
                self.dataBuffer = []
                print "Offset: "+str(self.minVal)
                #logger.info("Offset: "+str(self.minVal))
            self.totalPressure = self.totalPressure - self.minVal

    def callback(self, data):
        self.convert_and_publish_binary(data)

if __name__ == '__main__':

    measurement = Measurement()
    logger.info("Start measuring")

    rospy.init_node('Insole_Measurement', anonymous=False)
    rospy.Subscriber('/insole_rawdata', InsoleRawData, measurement.callback)
    rospy.spin()
