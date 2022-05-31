#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, select, time
servo_serial=None

class serial_data:
    myself=99
    buffer=[]
#    p=0


    def write(self, data, l):
        for i in range(l):
            if(l == 1):
                self.buffer.append(data)
            else:
                self.buffer.append(data[i])
            #self.p+=1
        return i
#//    while(read(fd, in_buf, in_buf_len) > 0);
            
    def read(in_buf, l):#return number of bytes read
            for i in range(l):
                j=l-i
                a=self.buffer[j-1]
                in_buf.append(a)
                self.buffer.pop()
                self.p-=1
                if self.p < 0:
                    return(i+1)
            return i+1
    def __init__(self):
        self.p=0


class servo_serial_data(serial_data):
    def __init__(self):
        super().__init__()
        self.buffer=[]
        myself=self
class motor_serial_data(serial_data):
    def __init__(self):
        super().__init__()
        self.buffer=[]
        
        

