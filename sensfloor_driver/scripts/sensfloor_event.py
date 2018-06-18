#!/usr/bin/env python

import rospy
import logging
import time
from ros_messages.msg import CapacitanceCluster, CapacitancePoint, Point as BeamerPoint, deleteDrawable
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Int32
from ros_messages.msg import SensfloorEventMsg

# a Sensor value of NEUTRAL_SENSOR_VALUE+SENSOR_COLOUR_RANGE will be coloured in full red
SENSOR_COLOUR_RANGE = 30.0

# Sensor value indicating that there is no weight on the sensor
NEUTRAL_SENSOR_VALUE = 128

RVIZ_MARKER_SCALE = 0.1
RVIZ_MARKER_LIFETIME = rospy.Duration()
BEAMER_MARKER_SCALE = 0.02

markerPub = rospy.Publisher('sensfloor_marker', Marker, queue_size=1)
footpressEvent = rospy.Publisher('sensfloor/event', SensfloorEventMsg, queue_size=1)

logger = logging.getLogger('Event-Sensfloor')


class SensfloorMarker(object):

    def __init__(self, header=None, points=None):
        #self.init_logger()

        self.header = header
        self.points = points
        self.locked = True
        self.dict = dict()
        self.actualPoints = list()
        self.isEvent = False

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

    def publish_marker(self):
        if self.points is not None or not self.locked:

            del self.actualPoints[:]
            for x in self.points:

                if not self.dict.has_key((x.x, x.y)) and x.c > 137:
                    self.isEvent = True
                    self.dict.update({(x.x, x.y): x.c})

                if x.c <= 137 and self.dict.has_key((x.x, x.y)):
                    self.dict.pop((x.x, x.y), None)

            if self.isEvent is True:
                event_time = int(round(time.time() * 1000))
                #print "Event-Sensfloor: " + str(event_time)
                logger.debug(event_time)
                footpressEvent.publish(SensfloorEventMsg(x=x.x, y=x.y, c=x.c, ms=event_time))
                self.isEvent = False


            for key, value in self.dict.iteritems():
                self.actualPoints.append(CapacitancePoint(x=key[0], y=key[1], c=value))

            marker = Marker()
            marker.header = self.header
            marker.ns = 'sensfloor'
            marker.type = Marker.POINTS
            marker.lifetime = RVIZ_MARKER_LIFETIME
            marker.pose.orientation.w = 1
            marker.scale.x = RVIZ_MARKER_SCALE
            marker.scale.y = RVIZ_MARKER_SCALE

            marker.id = 0
            marker.points = map(lambda x: Point(x=x.x, y=x.y), self.actualPoints)
            marker.colors = map(
                lambda x: ColorRGBA(r=max(0.01, min(1.0, float(x.c - NEUTRAL_SENSOR_VALUE) / SENSOR_COLOUR_RANGE)), g=0.0,
                                    b=0.0, a=max(0.01, min(1.0, float(x.c - NEUTRAL_SENSOR_VALUE) / SENSOR_COLOUR_RANGE))),
                self.actualPoints)

            markerPub.publish(marker)
            self.locked = True

    def callback(self, data):
        self.header = data.header
        self.points = data.points
        self.locked = False
        self.publish_marker()

if __name__ == '__main__':

    sensfloorMarker = SensfloorMarker()
    logger.info("Start Event Observer")

    rospy.init_node('SensfloorEvent', anonymous=True)
    rospy.Subscriber('/SensfloorArray', CapacitanceCluster, sensfloorMarker.callback)
    rospy.spin()
