#!/usr/bin/env python

import rospy
import time
from ros_messages.msg import InsoleEventMsg
from ros_messages.msg import SensfloorEventMsg
from ros_messages.msg import SensoleEventMsg

sensoleEvent = rospy.Publisher('sensole/match', SensoleEventMsg, queue_size=1)


class EventMatcher(object):

    def __init__(self):
        self.insoleEventList = list()
        self.sensfloorEventList = list()

    def publish_matched_events(self):
        while True:
            if len(self.sensfloorEventList) is not 0 and len(self.sensfloorEventList) is not 0:
                for iel in self.insoleEventList:
                    for sel in self.sensfloorEventList:
                        print(str((sel.ms-iel.ms)))
                        if -150 <= (sel.ms-iel.ms) <= 150:
                            print "MATCH!!! PARTYTIME!!!"
                        self.sensfloorEventList.remove(sel)
                    self.insoleEventList.remove(iel)
                        #elif self.sensfloorEventList.index(sel) == len(self.sensfloorEventList)-1:
                        #    self.sensfloorEventList.remove(sel)


    def insoleEventCallback(self, data):
        self.insoleEventList.append(data)

    def sensfloorEventCallback(self, data):
        self.sensfloorEventList.append(data)

if __name__ == '__main__':

    eventMatcher = EventMatcher()
    rospy.init_node('SensoleEvent', anonymous=True)
    rospy.Subscriber('insole/event', InsoleEventMsg, eventMatcher.insoleEventCallback)
    rospy.Subscriber('sensfloor/event', SensfloorEventMsg, eventMatcher.sensfloorEventCallback)
    eventMatcher.publish_matched_events()
    rospy.spin()
