#!/usr/bin/env python

import rospy
import time
from ros_messages.msg import CapacitanceCluster, CapacitancePoint, deleteDrawable
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Int32
import time
from sets import Set

import numpy as np
from sklearn.cluster import DBSCAN


# timeout variable can be omitted, if you use specific value in the while condition
timeout = 1   # [seconds]

# a Sensor value of NEUTRAL_SENSOR_VALUE+SENSOR_COLOUR_RANGE will be coloured in full red
SENSOR_COLOUR_RANGE = 30.0

# Sensor value indicating that there is no weight on the sensor
NEUTRAL_SENSOR_VALUE = 128

RVIZ_MARKER_SCALE = 0.1
RVIZ_MARKER_LIFETIME = rospy.Duration()

markerPub = rospy.Publisher('SensfloorCluster', Marker, queue_size=1)

class SensfloorMarker(object):

    def __init__(self):

        self.header = None
        self.points = None
        self.actualPoints = set()
        self.timeout_start = time.time()
        self.timeout_start_rec = time.time()


    def publish_marker(self):
        if self.points is not None:

            # set
            points_without_noise = self.remove_noise()


            if time.time() >= self.timeout_start + timeout:

                clusteredDataset = DBSCAN(eps=0.25, min_samples=1).fit(map(lambda x: (x[0], x[1]), points_without_noise))
                # set of tuples
                centres = self.calculate_centres(points_without_noise, clusteredDataset)
                self.actualPoints=self.actualPoints.union(centres)

                self.publish_centres(self.actualPoints)


                self.check_passed_time_and_delete_set(10)

    def check_passed_time_and_delete_set(self, timeLimit):
        if time.time() >= self.timeout_start_rec + timeLimit:
            self.actualPoints = set()
            self.timeout_start_rec = time.time()
        self.timeout_start = time.time()

    def publish_centres(self, centres):
        marker = self.init_marker(centres)
        markerPub.publish(marker)

    def calculate_centres(self, points, clusteredDataset):
        centres = set()
        pointsWithLabels = zip(points, clusteredDataset.labels_)
        for point, label in pointsWithLabels:
            centre = point
            pointsWithLabels.remove((point, label))
            for point2, label2 in pointsWithLabels:
                if label == label2:
                    centre = map(lambda x: x / 2, map(sum, zip(centre, point2)))
                    pointsWithLabels.remove((point2, label2))
            centres.add(tuple(centre))
        return centres

    def init_marker(self, centres):
        marker = Marker()
        marker.header = self.header
        marker.ns = 'sensfloor_cluster'
        marker.type = Marker.POINTS
        marker.lifetime = RVIZ_MARKER_LIFETIME
        marker.pose.orientation.w = 1
        marker.scale.x = RVIZ_MARKER_SCALE
        marker.scale.y = RVIZ_MARKER_SCALE
        marker.id = 0
        marker.points = map(lambda x: Point(x=x[0], y=x[1] - 1), centres)
        marker.colors = map(lambda x: ColorRGBA(r=0, g=100, b=0, a=1), centres)
        marker.text = str(centres)

        return marker

    def remove_noise(self):
        pointsWithoutNoises = set()

        for x in self.points:
            if x.c >= 150:
                pointsWithoutNoises.add((x.x, x.y))
        return list(pointsWithoutNoises)

    def callback(self, data):
        self.header = data.header
        self.points = data.points
        self.publish_marker()

    def int_to_colors(self, value):
        color = {0: (32, 0, 55),
                 1: (122, 54 ,0),
                 2: (56,0,98),
                 3: (34,56, 0),
                 4: (110,44,23),
                 5: (0,50, 26),
                 6: (100,200,95),
                 7: (200,100,26),
                 8: (95,100,200),
                 9: (200,37,100),
                 10: (100,42,200),
                 11: (3, 200,100),
                 12: (0,50,100),
                 13: (50,0,100),
                 14: (0,100,50),
                 15: (50,100,0),
                 16: (100,50,0),
                 17: (100,0,50),
                 18: (20,80,160),
                 19: (160,20,80),
                 20: (22,2,67),
                 21: (0, 255 ,0),
                 22: (44,0,33),
                 23: (55,6, 0),
                 24: (0,0,0),
                 25: (0,50, 77),
                 26: (100,200,6),
                 27: (200,100,5),
                 28: (5,100,200),
                 29: (200,3,100),
                 30: (100,2,200),
                 31: (88, 200,100),
                 32: (0,50,100),
                 33: (50,0,100),
                 34: (0,100,50),
                 35: (50,100,0),
                 36: (100,50,0),
                 37: (100,0,50),
                 38: (20,80,160),
                 39: (160,20,80),
                 -1: (1,56,6)
                 }

        return color[value]

if __name__ == '__main__':

    sensfloorMarker = SensfloorMarker()

    rospy.init_node('SensfloorEvent', anonymous=True)
    rospy.Subscriber('/SensfloorArray', CapacitanceCluster, sensfloorMarker.callback)
    rospy.spin()
