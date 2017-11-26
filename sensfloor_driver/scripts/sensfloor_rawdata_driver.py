#!/usr/bin/env python

import socket
import rospy
from ros_messages.msg import CapacitanceCluster, CapacitancePoint, Point as BeamerPoint, deleteDrawable
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import atexit

# a Sensor value of NEUTRAL_SENSOR_VALUE+SENSOR_COLOUR_RANGE will be coloured in full red
SENSOR_COLOUR_RANGE = 30.0

# Sensor value indicating that there is no weight on the sensor
NEUTRAL_SENSOR_VALUE = 128


TCP_IP = '10.1.4.40'
TCP_PORT = 5000
BUFFER_SIZE = 17
SENSOR_OFFSETS = [(0.416666656733, 0.166666671634), (0.333333343267, 0.0833333358169),
                  (0.166666671634, 0.0833333358169), (0.0833333358169, 0.166666671634),
                  (0.0833333358169, 0.333333343267), (0.166666671634, 0.416666656733), (0.333333343267, 0.416666656733),
                  (0.416666656733, 0.333333343267)]
RVIZ_MARKER_SCALE = 0.1
RVIZ_MARKER_LIFETIME = rospy.Duration()
BEAMER_MARKER_SCALE = 0.02

capDataPub = rospy.Publisher('SensfloorArray', CapacitanceCluster, queue_size=1)
markerPub = rospy.Publisher('SensfloorMarker', Marker, queue_size=1)

def publish_marker(points, header, marker_id):
    marker = Marker()
    marker.header = header
    marker.ns = 'sensfloor'
    marker.type = Marker.POINTS
    marker.lifetime = RVIZ_MARKER_LIFETIME
    marker.pose.orientation.w = 1
    marker.scale.x = RVIZ_MARKER_SCALE
    marker.scale.y = RVIZ_MARKER_SCALE

    marker.id = marker_id

    marker.points = map(lambda x: Point(x=x.x, y=x.y), points)
    marker.colors = map(
        lambda x: ColorRGBA(r=max(0.01, min(1.0, float(x.c - NEUTRAL_SENSOR_VALUE) / SENSOR_COLOUR_RANGE)), g=0.0,
                            b=0.0, a=max(0.01, min(1.0, float(x.c - NEUTRAL_SENSOR_VALUE) / SENSOR_COLOUR_RANGE))),
        points)

    # print marker.colors

    markerPub.publish(marker)


if __name__ == '__main__':
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    print "Sensfloor initiiert!"
    rospy.init_node('sensfloor_rviz', anonymous=True)
    # register a function to close the socket which is called if the node quits or crashes.
    # no correctly closing the connection results in reconnection issues
    def close_socket():
        s.close()


    atexit.register(close_socket)

    while not rospy.is_shutdown():
        data = s.recv(BUFFER_SIZE)
        # all sensfloor messages are 17 bytes long

        # the seventh by is the meaning byte, it tells you what kind of message it is
        meaningBytes = data[7].encode("hex")
        # print "meaning byte(s):", meaningBytes

        # we only care about messages with meaning byte == '11' since that means it is a sensor message
        if meaningBytes != '11':
            continue

        # identification bytes.
        # identificationBytes = data[0].encode("hex")
        # print "identification byte(s):", identificationBytes

        # Room ID is the same for sensfloor devices in the same installation
        # roomID = data[1:3].encode("hex")
        # print "roomID byte(s):", roomID

        # next two byte enumerate the devices in a room
        # the modules are on a grid

        # module number is the position of the device in its row
        moduleNumber = int(data[3].encode("hex"), 16)
        # print "module Number:", moduleNumber

        # row number is the row of the device
        rowNumber = int(data[4].encode("hex"), 16)
        # print "row Number:", rowNumber

        # position of the "lower left corner" of the module
        modulePosition = ((float(rowNumber - 1) * 0.5), (float(10 - moduleNumber) * 0.5))
        # print "module position:", modulePosition

        # every module contains eight sensor which are in  clock-wise order
        # the sensors divide the 50x50 cm area of each module in eight triangles
        # they measure the weight on each triangle

        # compute the position of each sensor
        # SensorPositions = map(lambda x: (x[0] + modulePosition[0], x[1] + modulePosition[1]),
        #                     SENSOR_OFFSETS)

        SensorPositions = list()

        for i in xrange(0, 8):
            SensorPositions.append((modulePosition[0] + SENSOR_OFFSETS[i][0], modulePosition[1] + SENSOR_OFFSETS[i][1]))

        # print SensorPositions

        # get the capacitance of each sensor
        SensorCapacitances = list()
        SensorCapacitances.append(int(data[9].encode("hex"), 16))
        SensorCapacitances.append(int(data[10].encode("hex"), 16))
        SensorCapacitances.append(int(data[11].encode("hex"), 16))
        SensorCapacitances.append(int(data[12].encode("hex"), 16))
        SensorCapacitances.append(int(data[13].encode("hex"), 16))
        SensorCapacitances.append(int(data[14].encode("hex"), 16))
        SensorCapacitances.append(int(data[15].encode("hex"), 16))
        SensorCapacitances.append(int(data[16].encode("hex"), 16))
        # print "data bytes(s):", capacitance

        # print "------------------------------------------"

        # create ros msg and fill it
        msg = CapacitanceCluster()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'sensfloor'

        msg.points = map(lambda x, y: CapacitancePoint(x=x[0], y=x[1], c=y), SensorPositions, SensorCapacitances)

        # unique id for each module, computed from its position
        moduleID = (rowNumber * 100) + moduleNumber
        # print 'moduleID:', moduleID

  #      publish_beamer(msg.points, moduleID)
        publish_marker(msg.points, msg.header, moduleID)
        capDataPub.publish(msg)
