#!/usr/bin/env python

import rospy
import time
import logging
from ros_messages.msg import InsolePressure
from ros_messages.msg import InsoleEventMsg

logger = logging.getLogger('Event-Insole   ')
footpressEvent = rospy.Publisher('insole/event', InsoleEventMsg, queue_size=1)


class InsoleEvent(object):

    def __init__(self):
        self.init_logger()
        self.totalPressure = 0
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

    def publish_insole_event(self):

        if self.totalPressure > 500 and not self.locked:
            #print self.totalPressure

            event_time = int(round(time.time() * 1000))
            footpressEvent.publish(InsoleEventMsg(press=self.totalPressure, ms=event_time))
            #print "Event-Insole:    "+str(event_time)
            logger.debug(event_time)
            logger.debug(self.totalPressure)
            self.locked = True

        if self.totalPressure < 500 and self.locked:
            self.locked = False

    def callback(self, data):
        self.totalPressure = data.areaTotal
        self.publish_insole_event()

if __name__ == '__main__':

    insoleEvent = InsoleEvent()
    logger.info("Start Event Observer")
    rospy.init_node('InsoleEvent', anonymous=True)
   # rospy.Subscriber('/R01/insole_pressure', InsolePressure, insoleEvent.callback)
    rospy.Subscriber('/insole_pressure', InsolePressure, insoleEvent.callback)
    rospy.spin()
