#!/usr/bin/env python

import rospy
import time
import logging
from ros_messages.msg import InsoleEventMsg
from ros_messages.msg import SensfloorEventMsg
from ros_messages.msg import SensoleEventMsg


sensoleEvent = rospy.Publisher('sensole/match', SensoleEventMsg, queue_size=1)


class Match(object):

    def publish_event_match(self):

        if self.totalPressure > 500 and not self.locked:
            sensoleEvent.publish(Int32(data=200))
            event_time = str(int(round(time.time() * 1000)))
            print "Event-Insole:    "+event_time
            self.locked = True

        if self.totalPressure < 500 and self.locked:
            self.locked = False


    def insoleEventCallback(self, data):
        self.totalPressure = data.areaTotal
        self.publish_event_match()

    def sensfloorEventCallback(self, data):
        self.totalPressure = data.areaTotal
        self.publish_event_match()


if __name__ == '__main__':

    matchEvent = MatchEvents()
    logger.info("Start Event Observer")
    rospy.init_node('SensoleEvent', anonymous=True)
    rospy.Subscriber('insole/event', InsoleEventMsg, insoleEventCallback)
    rospy.Subscriber('sensfloor/event', SensfloorEventMsg, sensfloorEventCallback)
    rospy.spin()
