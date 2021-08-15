#!/usr/bin/env python
#-*- coding: utf-8 -*-#

import nxt
import usb
from time import sleep

# Connecting to NXT1 brick
NXT1IsConnected = False
global b1
while not NXT1IsConnected:
    try:
        print("Connecting to NXT1...")
        b1 = nxt.locator.find_one_brick(name="NXT1")
        NXT1IsConnected = True
        print("NXT1 connected")
    except usb.core.USBError as e:
        if e.errno == 110:
            continue
        else:
            raise usb.core.USBError

# Resetting NXT1 motors and sensors
NXT1IsReset = False
while not NXT1IsReset:
    try:
        print("Resetting NXT1...")
        b1.start_program('NXT1_calibrate.rxe')
        sleep(10)
        try:
            b1.stop_program()
        except nxt.error.DirProtError as e:
            continue
        NXT1IsReset = True
        print("...Done")
    except usb.core.USBError as e:
        if e.errno == 110:
            continue
        else:
            raise usb.core.USBError

exit()