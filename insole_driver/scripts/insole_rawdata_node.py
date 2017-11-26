#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pygatt #dont use version 3.2.0 or higher
from binascii import hexlify
import rospy
import time
import logging
from threading import Thread
from ros_messages.msg import InsoleRawData
from ros_messages.msg import BleDevices

logging.basicConfig()
logging.getLogger('pygatt').setLevel(logging.INFO)

class InsoleSubcriber(object):

    def __init__(self, name, address):
        self.name = name
        self.address = address
        self.deviceName = ""
        self.adapter = None

    def start_read_insole_rawdata(self):

        self.deviceName = self.name.replace("Thorsis Ins. ESS ","")

        self.adapter = pygatt.GATTToolBackend(hci_device='hci0')
        self.adapter.start(True)

        print "Connecting Device "+self.deviceName
        while 1:
            try:
                time.sleep(2)
                self.device = self.adapter.connect(self.address, 10)
                time.sleep(2)
                break
            except:
                print "Error: reconnecting"
                self.adapter.disconnect

        self.adapter.sendline('mtu 64')
        time.sleep(2)

        self.callibrateSensor()

        print "Subscribe "+self.deviceName
        self.device.subscribe('00002b00-0000-1000-8000-00805f9b34fb', self.callback_func)

    def callback_func(self, handle, rawData):
        binaryData = '{}'.format(hexlify(str(rawData)))
        self.bD_lE = self.littleEndian(binaryData)
        insoleRawDataMsg = InsoleRawData(deviceName=self.deviceName, rawData=self.bD_lE)
        rawdataPub.publish(insoleRawDataMsg)

    def callibrateSensor(self):
        print "Callibrating Sensor for "+self.deviceName
        self.device.char_write_handle(0x0055, [1], True, None)

    def setSamplingRate(self):
        self.adapter.sendline('char-write-req 0x006f 1F000100')

    def littleEndian(self, _orig):
        leRawData = ''.join(sum([(c, d, a, b) for (a, b, c, d) in zip(*[iter(_orig[0:44])] * 4)], ()))
        leRawData = leRawData + ''.join(sum([(a, b) for (a, b) in zip(*[iter(_orig[44:50])] * 2)], ()))

        return leRawData

def callback(data):

    names = data.name
    addresses = data.address
    theadList = list()

    for name, address in zip(names, addresses):
        insolSub = InsoleSubcriber(name, address)

        t = Thread(target=insolSub.start_read_insole_rawdata())
        t.start()
        theadList.append(t)

    while 1:
        pass

def subscribe_ble_dev_addr():
        rospy.init_node('Insole_Rawdata', anonymous=True)
        rospy.Subscriber('ble_devices_address', BleDevices, callback)
       # rospy.spin()

if __name__ == '__main__':

    rawdataPub = rospy.Publisher('/insole_rawdata', InsoleRawData, queue_size=1)
    subscribe_ble_dev_addr()
