#!/usr/bin/env python

import socket
import time


TCP_IP = '127.0.0.1'
TCP_PORT = 6556

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP,TCP_PORT))
s.send('$TrackingAntennaGUI INIT')
time.sleep(0.1)
s.send('$startrecording 1 1 1 1 1 1 1')

print "START RECORDING"

