#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pygatt #dont use version 3.2.0 or higher
from ros_messages.msg import BleDevices
import logging

logging.basicConfig()
logging.getLogger('pygatt').setLevel(logging.DEBUG)

if __name__ == "__main__":
    try:
        adapter = pygatt.GATTToolBackend(hci_device='hci0')
        adapter.reset()
        devicePub = rospy.Publisher('ble_devices_address', BleDevices, queue_size=1)
        rospy.init_node('BLE_Devices_Address', anonymous=False)

        nameList = list()
        addressList = list()

        for device in adapter.scan(timeout=20, run_as_root=True):

            if device['name'] is not None:
                name = device['name'].encode('utf-8')
                address = device['address'].encode('utf-8')
                print "Found Device "+name
                print "with Address "+address
                if "Thorsis Ins." in name and name not in nameList:
                    nameList.append(name)
                    addressList.append(address)

        devices = BleDevices(name=nameList, address=addressList)
        adapter.disconnect

        while 1:
            devicePub.publish(devices)



    except rospy.ROSInterruptException:
        pass
