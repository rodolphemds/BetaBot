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
        b2 = nxt.locator.find_one_brick(name="NXT2")
        NXT2IsConnected = True
    except usb.core.USBError as e:
        if e.errno == 110:
            continue
        else:
            raise usb.core.USBError

# Running ligth_anim program
NXT2IsReset = False
while not NXT2IsReset:
    try:
        b2.start_program('NXT2_light_anim.rxe')
        NXT2IsReset = True
    except usb.core.USBError as e:
        if e.errno == 110:
            continue
        else:
            raise usb.core.USBError

exit()