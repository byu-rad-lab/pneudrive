#!/usr/bin python
import os
import time

ip_address = "192.168.0.188"

time.sleep(1.0)
os.system('mkdir /home/ubuntu/phildif')
os.system('ifconfig eth0 ' + ip_address + ' netmask 255.255.255.0')
