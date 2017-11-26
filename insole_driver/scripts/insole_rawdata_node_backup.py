#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pygatt #dont use version 3.2.0 or higher
from binascii import hexlify
import rospy
import time
import logging
from ros_messages.msg import InsoleRawData
from ros_messages.msg import BleDevices

logging.basicConfig()
logging.getLogger('pygatt').setLevel(logging.DEBUG)

class InsoleSubcriber(object):

    def __init__(self):

        self.rawdataPub = rospy.Publisher('insole_rawdata', InsoleRawData, queue_size=1)
        self.names = ["",""]

    def start_read_insole_rawdata(self, data):
        self.adapter1 = pygatt.GATTToolBackend(hci_device='hci0')
        self.adapter1.start(True)
        self.adapter2 = pygatt.GATTToolBackend(hci_device='hci0')
        self.adapter2.start(True)
        self.names[0] = data.name[0].replace("Thorsis Ins. ESS ","")
        self.names[1] = data.name[1].replace("Thorsis Ins. ESS ","")

        print "Connecting "+self.names[0]+" "+data.address[0]
        while 1:
            try:
                time.sleep(2)
                self.device1 = self.adapter1.connect(data.address[0], 10)
                time.sleep(2)
                break
            except:
                print "Error: reconnecting"
                self.adapter1.disconnect

        print "Connecting "+self.names[1]+" "+data.address[1]
        while 1:
            try:
                self.device2 = self.adapter2.connect(data.address[1], 10)
                time.sleep(2)
                break
            except:
                print "Error: reconnecting"
                self.adapter2.disconnect

        self.adapter1.sendline('mtu 64')
        time.sleep(2)
        self.adapter2.sendline('mtu 64')
        time.sleep(2)

        self.callibrateSensor()

        print "Subscribe "+self.names[0]
        self.device1.subscribe("00002b00-0000-1000-8000-00805f9b34fb", self.callback_func1)
        print "Subscribe "+self.names[1]
        self.device2.subscribe("00002b00-0000-1000-8000-00805f9b34fb", self.callback_func2)

        print "Subscription for all devices completed"

        while 1:
            pass #hier Einstiegspunkt fuer Geraete An- und Abmeldung moeglich

    def callback_func1(self, handle, rawData):
        binaryData = '{}'.format(hexlify(str(rawData)))
        self.bD_lE = self.littleEndian(binaryData)
        insoleRawDataMsg = InsoleRawData()
        insoleRawDataMsg.header.stamp = rospy.Time.now()
        insoleRawDataMsg.header.frame_id = 'insole'
        insoleRawDataMsg.deviceName= self.names[0]
        insoleRawDataMsg.rawData = self.bD_lE
        self.rawdataPub.publish(insoleRawDataMsg)

    def callback_func2(self, handle, rawData):
        binaryData = '{}'.format(hexlify(str(rawData)))
        self.bD_lE = self.littleEndian(binaryData)
        insoleRawDataMsg = InsoleRawData()
        insoleRawDataMsg.header.stamp = rospy.Time.now()
        insoleRawDataMsg.header.frame_id = 'insole'
        insoleRawDataMsg.deviceName= self.names[1]
        insoleRawDataMsg.rawData = self.bD_lE
        self.rawdataPub.publish(insoleRawDataMsg)

    def littleEndian(self, _orig):
       # print "O: "+_orig
        leRawData = ''.join(sum([(c, d, a, b) for (a, b, c, d) in zip(*[iter(_orig[0:44])] * 4)], ()))
        leRawData = leRawData + ''.join(sum([(a, b) for (a, b) in zip(*[iter(_orig[44:50])] * 2)], ()))
       # print "B: "+leRawData

        return leRawData

    def callibrateSensor(self):
        self.device1.char_write_handle(0x0055, [1], True, None)
        self.device2.char_write_handle(0x0055, [1], True, None)

    def subscribe_ble_dev_addr(self):
        rospy.init_node('Insole_Rawdata', anonymous=True)
        rospy.Subscriber('ble_devices_address', BleDevices, self.start_read_insole_rawdata)
        rospy.spin()

if __name__ == '__main__':
    insoleSub = InsoleSubcriber()
    insoleSub.subscribe_ble_dev_addr()
