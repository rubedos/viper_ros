#!/usr/bin/python

from socket import *
import sys

port=45454
try:
    print('\nStarting VIPER discovery (wait time ~1-2mins)...\n')
    s=socket(AF_INET, SOCK_DGRAM)
    s.bind(('',port))
    while True:
        data,address=s.recvfrom(1024);
        #print((data.decode('utf-8')).rstrip());
        #VIPER|id=viper?|name=viper?|IP=192.168.1.79|serial=s/n?|version= 1.0.0.1796|DATE=2018.12.28-11:46:01|ENDOFLINE
        viperName = data.split('|')[2].split('=')[1];
        viperIp = data.split('|')[3].split('=')[1];
        viperSerial = data.split('|')[4].split('=')[1];
        viperTime = data.split('|')[6].split('=')[1];
        str1 = "VIPER=" + viperName + " http://" + viperIp + " time=" + viperTime
        print str1
    s.close()

except (KeyboardInterrupt, SystemExit):
    s.close()
    print('Exit on Keyboard Interrupt')