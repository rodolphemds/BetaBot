#!/usr/bin/env python
#-*- coding: utf-8 -*-#

import nxt
import usb
from time import sleep
# Connecting to NXT2 brick
NXT2IsConnected = False
global b2
while not NXT2IsConnected:
    try:
        print("Connecting to NXT2...")
        b2 = nxt.locator.find_one_brick(name="NXT2")
        NXT2IsConnected = True
        print("NXT2 connected")
    except usb.core.USBError as e:
        if e.errno == 110:
            continue
        else:
            raise usb.core.USBError

# Resetting NXT2 motors and sensors
NXT2IsReset = False
while not NXT2IsReset:
    try:
        print("Resetting NXT2...")
        b2.start_program('NXT2_calibrate.rxe')
        sleep(25)
        try:
            b2.stop_program()
        except nxt.error.DirProtError as e:
            continue
        NXT2IsReset = True
        print("...Done")
    except usb.core.USBError as e:
        if e.errno == 110:
            continue
        else:
            raise usb.core.USBError

exit()