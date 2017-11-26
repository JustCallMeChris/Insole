#!/usr/bin/env python

import rospy
from ros_messages.msg import CapacitanceCluster, CapacitancePoint, Point as BeamerPoint, deleteDrawable
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sklearn.cluster import KMeans


# a Sensor value of NEUTRAL_SENSOR_VALUE+SENSOR_COLOUR_RANGE will be coloured in full red
SENSOR_COLOUR_RANGE = 30.0

# Sensor value indicating that there is no weight on the sensor
NEUTRAL_SENSOR_VALUE = 128

RVIZ_MARKER_SCALE = 0.1
RVIZ_MARKER_LIFETIME = rospy.Duration()
BEAMER_MARKER_SCALE = 0.02

markerPub = rospy.Publisher('sensfloor_marker', Marker, queue_size=1)




class SensfloorMarker(object):

    def __init__(self, header=None, points=None):
        self.header = header
        self.points = points

    def publish_marker(self):
        if self.points is None:

            print "None"


        else:

            for list in self.points:
                print "x: "+str("%.2f" % list.x)
                print "y: "+str("%.2f" % list.y)
                print "-----------------"


            marker = Marker()
            marker.header = self.header
            marker.ns = 'sensfloor'
            marker.type = Marker.POINTS
            marker.lifetime = RVIZ_MARKER_LIFETIME
            marker.pose.orientation.w = 1
            marker.scale.x = RVIZ_MARKER_SCALE
            marker.scale.y = RVIZ_MARKER_SCALE

            marker.id = 0

            marker.points = map(lambda x: Point(x=x.x, y=x.y), self.points)
            marker.colors = map(
                lambda x: ColorRGBA(r=max(0.01, min(1.0, float(x.c - NEUTRAL_SENSOR_VALUE) / SENSOR_COLOUR_RANGE)), g=0.0,
                                    b=0.0, a=max(0.01, min(1.0, float(x.c - NEUTRAL_SENSOR_VALUE) / SENSOR_COLOUR_RANGE))),
                self.points)

            markerPub.publish(marker)
            print "--------ENDE---------"

    def callback(self, data):
        self.header = data.header
        self.points = data.points


if __name__ == '__main__':

    sensfloorMarker = SensfloorMarker()

    rospy.init_node('SensfloorMarker', anonymous=True)
    rospy.Subscriber('/SensfloorArray', CapacitanceCluster, sensfloorMarker.callback)

    while not rospy.is_shutdown():

        sensfloorMarker.publish_marker()
