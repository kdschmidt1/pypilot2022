

#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, select, time
from gpiozero import CPUTemperature
from servo import *
from pypilot.arduino_servo.raspi_motorx import *
from pypilot.arduino_servo.myeeprom import *
from cmath import nan
from server import pypilotServer
from client import pypilotClient
from pypilot.arduino_servo.crc import *
from pylint.extensions.check_elif import ElseifUsedChecker
#from meld.vc._vc import call_temp_output
#TODO servo funktioniert nur negativ!!
#TODO Timeout handling,  // test fault pins,     // test current   2000sps @ 8mhz     // test voltage , // test over temp, test rudder

class CommandCodes(object):
        COMMAND_CODE=0xc7
        RESET_CODE=0xe7
        MAX_CURRENT_CODE=0x1e
        MAX_CONTROLLER_TEMP_CODE=0xa4
        MAX_MOTOR_TEMP_CODE=0x5a
        RUDDER_RANGE_CODE=0xb6
        RUDDER_MIN_CODE=0x2b
        RUDDER_MAX_CODE=0x4d
        REPROGRAM_CODE=0x19
        DISENGAGE_CODE=0x68
        MAX_SLEW_CODE=0x71
        EEPROM_READ_CODE=0x91
        EEPROM_WRITE_CODE=0x53
        CLUTCH_PWM_CODE=0x36
        def __init__(self, name):
            super(CommandCodes, self).__init__(name, 0)
class   results():
        CURRENT_CODE=0x1c
        VOLTAGE_CODE=0xb3
        CONTROLLER_TEMP_CODE=0xf9
        MOTOR_TEMP_CODE=0x48
        RUDDER_SENSE_CODE=0xa7
        FLAGS_CODE=0x8f
        EEPROM_VALUE_CODE=0x9a
        UNKNOWN_CODE=0x99            


class RaspberryServo:
# STATIC VARIABLES
    lastaddr=0
    lastvalue=0
    in_buf=[]
    packet_count=0
    def __init__(self):
        self.server = pypilotServer()
        self.client = pypilotClient(self.server)
        self.motor = RaspberryMotor(self.server)
        self.eeprom = myeeprom()
        self.max_current_value = 0
        self.voltage = self.current = False
        self.receivebuf=RaspberryMotor.sendbuf
        self.sendbuf=RaspberryMotor.receivebuf

        self.pollcount=0


        # sensors

        self.voltage=0 
        self.current=0 
        self.controller_temp=0 
        self.motor_temp=0 
        self.rudder=0

    #// parameters
        self.raw_max_current=0
        self.rudder_min=0
        self.rudder_max=0

    #// eeprom settings (some are parameters)
        self.max_current=0 
        self.max_controller_temp=0
        self.max_motor_temp=0
        self.rudder_range=0 
        self.rudder_offset=0 
        self.rudder_scale=0 
        self.rudder_nonlinearity=0
        self.max_slew_speed=0 
        self.max_slew_slow=0
        self.current_factor=0 
        self.current_offset=0 
        self.voltage_factor=0 
        self.voltage_offset=0

        self.min_speed=0 
        self.max_speed=0
        self.gain=0 
        self.clutch_pwm=0
    
        ##print('RaspberryServo _init_')
        self.in_sync_count = 0;
        self.out_sync = 0;
        self.in_buf=[]
        self.in_buf_len = 0;
        self.max_current = 0;
        self.params_set = 0;
        self.flags = 0;
        
        self.nosync_count = 0;
        self.nosync_data = 0;
        self.eeprom_read = 0;
        #// force unsync
        #self.fd_servo=servo_serial_data
        reset_code = [0xF1, 0xF2, 0xF3, 0xF4];
        self.serial_write4(self.sendbuf, reset_code, len(reset_code));
        self.in_buf=[]
        #c = servo_serial_data.read(self.in_buf, 4);
        test=99

    def serial_read4(self,buffer):
            count = 1
            if len(buffer)>0:
                a=buffer.pop(0)
                return a
            else:
                return -1
            return a
    def serial_write4(self,buffer,data,l):
        for i in range(l):
            a=len(buffer)
            if(l == 1):
                buffer.append(data)
            else:
                buffer.append(data[i])
        return i
        


    def command(self, command): # aus cpp
        self.timeout=0
        #print('RaspberryServo command ',command)
        command = min(max(command, -1), 1)
        self.raw_command((command+1)*1000)


    def process_packet(self,in_buf):
        #print('RaspberryServo process_packet (',self.pollcount, ')')
        if(self.packet_count < 255):
            self.packet_count+=1;
        value = in_buf[1] + (in_buf[2]<<8);
        code=in_buf[0]
        for i in range(4):
            in_buf.pop(-1)
        
        


        
        #self.flags |= ServoFlags.SYNC
        #code,value = self.motor.ret_val(self.pollcount)  # von RaspiMotor einlesen,  read only 1 parameter at a time
        #print('RaspberryServo process_packet(',self.pollcount,'),code=',ret.code,' value=',ret.value)
        #self.pollcount+=1
        #if(self.pollcount>42):
         #   self.pollcount=0
        #print('RaspberryServo process_packet ret= ',ret)
#if(packet_count < 255)
 #       packet_count++;
#    uint16_t value = in_buf[1] + (in_buf[2]<<8);

#    switch(in_buf[0]) {
#        print("RaspberryServo received CODE",code);

        if(code == results.CURRENT_CODE):
            self.current = value / 100.0;
            #//printf("servo current  %f\n", current);
            return ServoTelemetry.CURRENT
        if(code == results.VOLTAGE_CODE):
            self.voltage = value / 100.0;
            #//printf("servo voltage  %f\n", voltage);
            return ServoTelemetry.VOLTAGE;
        if(code == results.CONTROLLER_TEMP_CODE):
            self.controller_temp = value;
            #print("servo temp:  ", value);
            return ServoTelemetry.CONTROLLER_TEMP;
        if(code == results.MOTOR_TEMP_CODE):
            self.motor_temp = value / 100.0;
            return ServoTelemetry.MOTOR_TEMP;
        if(code == results.RUDDER_SENSE_CODE):
            if(value == 65535):
                self.rudder = nan;
            else:
                self.rudder = value / 65472.0 - 0.5; #// nominal range of -0.5 to 0.5

            return ServoTelemetry.RUDDER;
        if(code == results.FLAGS_CODE):
            self.flags = value | ServoFlags.SYNC

            if(self.flags & ServoFlags.INVALID):
                print("RaspberryServo received invalid packet (check serial connection)\n");
            return ServoTelemetry.FLAGS
        
        if(code == results.EEPROM_VALUE_CODE):
            #print("RaspberryServo EEPROM value: ", value);
            if self.eeprom_read<0:
                self.eeprom_read = 4;
            addr = value&0xff
            val= value >> 8 #in_buf[1], val = in_buf[2];
            #print("RaspberryServo EEPROM_VALUE_CODE addr", addr,'val:',val);
            if addr <len(list(self.eeprom.arduino)):
              #  print("self VALUE Verify :",list(self.eeprom.arduino)[addr],val);
              pass
            if addr==0:
                test=69
            if(addr&1):
                if(addr == RaspberryServo.lastaddr+1):
                    #print('RaspberryServo EEPROM_VALUE_CODE: call eeprom.value(',RaspberryServo.lastaddr,RaspberryServo.lastvalue,')')
                    self.eeprom.value(RaspberryServo.lastaddr, RaspberryServo.lastvalue);
                    self.eeprom.value(addr, val);
            else:
                RaspberryServo.lastaddr = addr;
                RaspberryServo.lastvalue = val;
            #print("RaspberryServo EEPROM_VALUE_CODE KDS??", addr,'val:',val);
            self.eeprom.initial()

        #// only report eeprom on initial read for all data
            if(self.eeprom.initial_read):    #self.eeprom.initial()):
                self.max_current = self.eeprom.get_max_current();
                self.max_controller_temp = self.eeprom.get_max_controller_temp();
                self.max_motor_temp = self.eeprom.get_max_motor_temp();
                self.rudder_range = self.eeprom.get_rudder_range();
                self.rudder_offset = self.eeprom.get_rudder_offset();
                self.rudder_scale = self.eeprom.get_rudder_scale();
                self.rudder_nonlinearity = self.eeprom.get_rudder_nonlinearity();
                self.max_slew_speed = self.eeprom.get_max_slew_speed();
                self.max_slew_slow = self.eeprom.get_max_slew_slow();
                self.current_factor = self.eeprom.get_current_factor();
                self.current_offset = self.eeprom.get_current_offset();
                self.voltage_factor = self.eeprom.get_voltage_factor();
                self.voltage_offset = self.eeprom.get_voltage_offset();
                self.min_speed = self.eeprom.get_min_speed();
                self.max_speed = self.eeprom.get_max_speed();
                self.gain = self.eeprom.get_gain();
                self.clutch_pwm = self.eeprom.get_clutch_pwm();
            
                self.params(60, 0, 1, self.max_current, self.max_controller_temp, self.max_motor_temp, self.rudder_range, self.rudder_offset, self.rudder_scale, self.rudder_nonlinearity, self.max_slew_speed, self.max_slew_slow, self.current_factor, self.current_offset, self.voltage_factor, self.voltage_offset, self.min_speed, self.max_speed, self.gain, self.clutch_pwm);
                return ServoTelemetry.EEPROM;
            elif(not self.eeprom.initial_read):
                #// if we got an eeprom value, but did not get the initial read,
                #// send a lot of disengage commands to speed up communication speed which
                #    // will complete reading eeprom faster

                #print('RaspberryServo EEPROM_VALUE_CODE HAS NOT GOT INITIAL READ')
                #self.disengage();
                pass
        return 0;

    def read_serial(self,buffer,in_buf,count):
            l=min(len(buffer), count)
            for i in range(l):
                a=buffer[0]
                in_buf.append(a)
                buffer.pop(0)
            return l


    def poll(self):
        #print("RaspberryServo poll\n");
        self.flags |= ServoFlags.SYNC
        self.flags |= ServoFlags.ENGAGED
       
        if (not(self.flags & ServoFlags.SYNC)): 
            self.disengage();
            self.nosync_count+=1;
            if(self.nosync_count >= 400 and not self.nosync_data):
                print("RaspberryServo fail no data\n");
                return -1
            if(self.nosync_count >= 1000):
                print("RaspberryServo fail sync\n");
                return -1;
        else:
            self.nosync_count = 0;
            self.nosync_data = 0;

        if(self.in_buf_len < 4):
            cnt = 255 - self.in_buf_len;#cnt = len(self.in_buf) - self.in_buf_len;
            while self.in_buf_len < 4:
                #c = self.fd.read(self.in_buf + self.in_buf_len, cnt);
                #self.servo_serial.write(reset_code, len(reset_code));
                c=self.serial_read4(self.receivebuf)
                if(c < 0):
                    break;
                else:
                    self.in_buf.append(c)
                    self.in_buf_len += 1;
                #c = self.read_serial(self.servo_motor.buffer, self.in_buf,cnt)
            #self.in_buf_len += c;
            if(self.in_buf_len < 4):
                return 0;
        
        ret=0
#        a=self.process_packet(self.pollcount);
#        ret |= a;

        while(self.in_buf_len >= 4):
        #uint8_t crc = crc8(in_buf, 3);
        #if(crc == in_buf[3]) { // valid packet
            if(self.in_sync_count >= 0):#2):
                ret |= self.process_packet(self.in_buf);
            else:
                self.in_sync_count+=1
            self.in_buf_len-=4;
            for i in range(self.in_buf_len):
                self.in_buf[i] = self.in_buf[i+4];
        #} else {
         #   // invalid packet, shift by 1 byte
         #   in_sync_count = 0;
         #   in_buf_len--;
         #   for(int i=0; i<in_buf_len; i++)
         #       in_buf[i] = in_buf[i+1];





        if (self.flags & ServoFlags.SYNC):
            return ret;

        if (ret):
            self.nosync_data = 1;
    
        return 0;



        #??????????????
        ##print('RaspberryServo poll: ',self.pollcount)

        if(self.flags & ServoFlags.ENGAGED):
            self.update_command(); #// update speed changes to slew speed
        ret=0
        #self.flags = 32768  # rebooted
        #self.flags |= 8 # ENGAGED
        self.flags |= ServoFlags.SYNC
        #self.flags &= ~ServoFlags.MAX_RUDDER_FAULT;

        #??????????????
    
    def fault(self):
        ##print('RaspberryServo fault: ',self.flags & ServoFlags.OVERCURRENT_FAULT)
        return self.flags & ServoFlags.OVERCURRENT_FAULT

    def params(self, _raw_max_current, _rudder_min, _rudder_max, _max_current, _max_controller_temp, _max_motor_temp, _rudder_range, _rudder_offset, _rudder_scale, _rudder_nonlinearity, _max_slew_speed, _max_slew_slow, _current_factor, _current_offset, _voltage_factor, _voltage_offset, _min_speed, _max_speed, _gain, _clutch_pwm):
        #print('RaspberryServo params to be done ')
        raw_max_current = min(60, max(0, _raw_max_current));
        self.rudder_min = min(.5, max(-.5, _rudder_min));
        self.rudder_max = min(.5, max(-.5, _rudder_max));
        #print("RaspberryServo params:", self.rudder_min,self.rudder_max);

        max_current = min(60, max(0, _max_current));
        self.eeprom.set_max_current(max_current);

        max_controller_temp = min(100, max(30, _max_controller_temp));
        self.eeprom.set_max_controller_temp(max_controller_temp);

        max_motor_temp = min(100, max(30, _max_motor_temp));
        self.eeprom.set_max_motor_temp(max_motor_temp);

        rudder_range = min(120, max(0, _rudder_range));
        self.eeprom.set_rudder_range(rudder_range);

        rudder_offset = min(500, max(-500, _rudder_offset));
        self.eeprom.set_rudder_offset(rudder_offset);

        rudder_scale = min(4000, max(-4000, _rudder_scale));
        self.eeprom.set_rudder_scale(rudder_scale);

        rudder_nonlinearity = min(4000, max(-4000, _rudder_nonlinearity));
        self.eeprom.set_rudder_nonlinearity(rudder_nonlinearity);

        max_slew_speed = min(100, max(0, _max_slew_speed));
        #print('RaspberryServo  MAX_SLEW_CODE _max_slew_speed: ',_max_slew_speed)


        self.eeprom.set_max_slew_speed(max_slew_speed);

        max_slew_slow = min(100, max(0, _max_slew_slow));
        self.eeprom.set_max_slew_slow(max_slew_slow);

        current_factor = min(1.2, max(.8, _current_factor));
        self.eeprom.set_current_factor(current_factor);

        current_offset = min(1.2, max(-1.2, _current_offset));
        self.eeprom.set_current_offset(current_offset);

        voltage_factor = min(1.2, max(.8, _voltage_factor));
        self.eeprom.set_voltage_factor(voltage_factor);

        voltage_offset = min(1.2, max(-1.2, _voltage_offset));
        self.eeprom.set_voltage_offset(voltage_offset);

        min_speed = min(100, max(0, _min_speed));
        self.eeprom.set_min_speed(min_speed);
    
        max_speed = min(100, max(0, _max_speed));
        self.eeprom.set_max_speed(max_speed);

        gain = min(10, max(-10, _gain));
        # disallow gain from -.5 to .5
        if(gain < 0):
            gain = min(gain, -.5);
        else:
            gain = max(gain, .5);
        self.eeprom.set_gain(gain);

        clutch_pwm = min(100, max(10, _clutch_pwm));
        self.eeprom.set_clutch_pwm(clutch_pwm);

        self.params_set = 1

    def send_value(self, command, value):   
        code = [command, int(value)&0xff, (int(value)>>8) & 0xff, 0];
        code[3] = crc8(code, 3);
        self.serial_write4(self.sendbuf,code, 4);

        #self.servo_serial.write(self.servo_serial, code, 4);
        
        #self.motor.process_packet(code, value)


        
    def send_params(self):   # aus cpp
        #// send parameters occasionally, but only after parameters have been
        #// initialized by the upper level
        # send all parameters at once

        if (not self.params_set):
            return;
        #print("RaspberryServo send_params",self.out_sync);

        if self.out_sync==0 or self.out_sync==8 or self.out_sync==16:
            self.send_value(CommandCodes.MAX_CURRENT_CODE, self.eeprom.local['max_current'])
        if self.out_sync==4:
            self.send_value(CommandCodes.MAX_CONTROLLER_TEMP_CODE, self.eeprom.local['max_controller_temp'])
        if self.out_sync==6:
            self.send_value(CommandCodes.MAX_MOTOR_TEMP_CODE, self.eeprom.local['max_motor_temp'])
        if self.out_sync==10:
            self.send_value(CommandCodes.CLUTCH_PWM_CODE, self.eeprom.local['clutch_pwm'])
        if self.out_sync==12:
            #print("RaspberryServo RUDDER_MIN_CODE:", self.rudder_min);   
            self.send_value(CommandCodes.RUDDER_MIN_CODE, (int)((self.rudder_min+0.5)*65472));
        if self.out_sync==14:
            #print("RaspberryServo RUDDER_MAX_CODE:", self.rudder_max);   
            self.send_value(CommandCodes.RUDDER_MAX_CODE, (int)((self.rudder_max+0.5)*65472));
            
        if self.out_sync==18:
            self.send_value(CommandCodes.MAX_SLEW_CODE, self.eeprom.local['max_slew_slow'] << 8 | self.eeprom.local['max_slew_speed'])
        
        if self.out_sync==20:
            #print("RaspberryServo EEPROM_READ1 self.eeprom_read:", self.eeprom_read);
            if(self.eeprom_read == 0): 
                end=0
                addr,end = self.eeprom.need_read(end);
                if(addr >= 0 and end > addr):
                    #print("RaspberryServo EEPROM_READ2 ", addr, end);
                    self.send_value(CommandCodes.EEPROM_READ_CODE, addr | end<<8);
                #print("RaspberryServo EEPROM_READ3", addr, end);
            else:
                self.eeprom_read-=1;
        if self.out_sync==22:
            addr = self.eeprom.need_write();#eeprom.local[i] != eeprom.arduino[i] and  eeprom.verified[k]
            if(addr >= 0):
                #print("RaspberryServo EEPROM NEED WRITE(",addr,")");
                i=0
                keys = list(self.eeprom.local)
                
                for key, value in self.eeprom.local.items():
                    #print(i,key, value)
                    i+=1
                
            #//printf("EEPROM_WRITE %d %d %d\n", addr, eeprom.data(addr), eeprom.data(addr+1));
            #// send two packets, always write 16 bits atomically
                #print("addr",addr,'i',i)
                testlo=self.eeprom.local[keys[int(addr/2)]]&0xff
                #print('lowbyte ', hex(testlo))
                testhi=(self.eeprom.local[keys[int(addr/2)]]>>8)& 0xff
                #print('highbyte ', hex(testhi))
                self.send_value(CommandCodes.EEPROM_WRITE_CODE, addr | testlo<<8);
                addr+=1;
                self.send_value(CommandCodes.EEPROM_WRITE_CODE, addr | testhi<<8);
        self.out_sync += 1
        if(self.out_sync >= 23):
            self.out_sync = 0;
    

    def raw_command(self, value): # aus cpp
        self.send_value(CommandCodes.COMMAND_CODE, value);
        self.send_params();
        return

    def reset(self):
        self.send_value(CommandCodes.RESET_CODE, 0);
        return 
    def disengage(self):
        ##print('RaspberryServo disengage')
        self.send_value(CommandCodes.DISENGAGE_CODE, 0);
        self.send_params()
        return
#???????????????????        

        self.timeout=30
        if(self.flags | 8): #ENGAGED
            self.flags &= ~8
        # 1. RESET auf LOW setzen GPIO17 PIN11
        # 2. CLUTCH auf LOW setzen GPIO13 PIN 33
        # 3. ENABLE LED auf HIGH setzen setzen GPIO22 PIN 15
        # 4. Fehlerleitungen PULLUP GPIO23,24 PIN 16,18 -> Jumper J5 auftrennen, dann bekommen die LEDs Spannung von den GPIOs
        #     Pins testen: sollten dann LOW haben wenn alles ok 
#???????????????????        

    def reprogram(self):
        self.send_value(CommandCodes.REPROGRAM_CODE, 0);

