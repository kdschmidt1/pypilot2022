#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# Boat imu is built on top of RTIMU

# it is an enhanced imu with special knowledge of boat dynamics
# giving it the ability to auto-calibrate the inertial sensors

import os, sys
import time, math, multiprocessing, select
import socket

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from client import pypilotClient
    from values import *

    from nonblockingpipe import NonBlockingPipe
except:
    import failedimports


class RaspiMotorx(object):
    def __init__(self, server):
        self.client = pypilotClient(server)
        self.multiprocessing = server.multiprocessing
        if self.multiprocessing:
            self.pipe, pipe = NonBlockingPipe('RaspiMotor', self.multiprocessing)
            self.process = multiprocessing.Process(target=self.process, args=(pipe,), daemon=True)
            self.process.start()
            return
        self.process = False
        self.setup()
          
    def setup(self):
        self.init()
        self.lastdata = False
        self.rate = 10

    def init(self):
        t0 = time.monotonic()
        time.sleep(.1)

    def process(self, pipe):
        print('RaspiMotorx process', os.getpid())

#        if os.system('sudo chrt -pf 2 %d 2>&1 > /dev/null' % os.getpid()):
#            print(_('warning, failed to make imu process realtime'))
#        else:
#            print(_('made imu process realtime'))

        self.setup()
        while True:
            t0 = time.monotonic()
            data = self.read()


            dt = time.monotonic() - t0
            period = 1/self.rate
            t = period - dt
            if t > 0 and t < period:
                time.sleep(t)
            else:
                print(_('RaspiMotorx process failed to keep time'), dt, t0, t1, t2, t3)

    def read(self):
        pass        
    def poll(self):
        pass        

class BoatRaspiMotorx(object):
    def __init__(self, client):
        self.client = client
        self.motor = RaspiMotorx(client.server)

    def __del__(self):
        #print('terminate imu process')
        #self.imu.process.terminate()
        pass

    def register(self, _type, name, *args, **kwargs):
        value = _type(*(['RaspiMotorx.' + name] + list(args)), **kwargs)
        return self.client.register(value)

    def BOATread(self):
        return self.imu.read()

    def poll(self):
        if not self.imu.multiprocessing:
            self.imu.poll()

    def read(self):
        print('BoatRaspiMotorxRead')


      
def main():
    from server import pypilotServer
    server = pypilotServer()
    client = pypilotClient(server)
    boatimu = BoatRaspiMotorx(client)

    quiet = '-q' in sys.argv

    lastprint = 0
    while True:
        t0 = time.monotonic()
        server.poll()
        client.poll()
        #print('data:',data['heading'])
        while True:
            dt = 1/BoatRaspiMotorx.rate.value - (time.monotonic() - t0)
            if dt < 0:
                break
            if dt > 0:
                time.sleep(dt)
            
if __name__ == '__main__':
    main()
