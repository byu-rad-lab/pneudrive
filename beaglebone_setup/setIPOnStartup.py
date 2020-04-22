#!/usr/bin python
import os
import time

time.sleep(1.0)
os.system('ifconfig eth0 192.168.0.188 netmask 255.255.255.0')
