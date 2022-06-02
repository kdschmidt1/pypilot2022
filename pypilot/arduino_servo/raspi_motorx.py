import re, os

#from pypilot.arduino_servo.raspberry_servo_python2 import servo_serial_data
#from pypilot.arduino_servo.raspberry_servo_python2 import motor_serial_data
from servo import *
from pypilot.arduino_servo.myeeprom import *


import board
#from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from timeloop import Timeloop
from datetime import timedelta
import time, math, multiprocessing, select
from server import pypilotServer
from client import pypilotClient
from pypilot.arduino_servo.crc import *
from gpiozero import CPUTemperature
try:
    from nonblockingpipe import NonBlockingPipe
except:
    print("Failed Import")

globalself=99
tltl = Timeloop()
my_self=66
xyz=89
#global TiSmeloopRetval
#TimeloopRetval=9999
#Timeloop_125=8888


#@tltl.job(interval=timedelta(seconds=0.5))
#def sample_job_2Hz():
#    global TimeloopRetval
#    #print('Timeloop: ',time.monotonic())
#    path='/sys/bus/w1/devices/28-00000d3c7f07/w1_slave'
#    #print('1-WIRE:',read_sensor(path))    
#    TimeloopRetval=read_sensor(path)
#    #print('TimeloopRetval: ',TimeloopRetval)
#    pass

    
i2c_bus = board.I2C()

#ina219 = INA219(i2c_bus)
ads = ADS.ADS1115(i2c_bus)
SHUNT_OHMS = 0.1
from gpiozero import Button,LED
from rpi_hardware_pwm import HardwarePWM
PWM_FREQENCY = 12000
CLUTCH_DELAY = 5    # sek bis pwm reduziert wird




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

#def hb_error(no):
 #   print('H-Bridge Error ', no)

import threading


class RaspberryMotor:
    sendbuf=[]
    receivebuf=[]
    def __init__(self, server):
        
        #self.server = pypilotServer()
        #self.client = pypilotClient(server)
        #self.multiprocessing = server.multiprocessing
        #x = threading.Thread(target=self.loop, args=(1,))
        #x.start()
        #return

        
        
        self.setup()
#        if self.multiprocessing:
#            self.pipe, pipe = NonBlockingPipe('Motor pipe', self.multiprocessing)
#            self.process = multiprocessing.Process(target=self.loop, args=(pipe,), daemon=True)
#            self.process.start()
#            return
#        self.process = False
        #self.setup()

    def setup(self):
        global xyz
        xyz=self

        self.myeeprom = myeeprom()
        self.myeeprom.eeprom_read_byte(5)
        self.v=0
        self.oldtime = time.monotonic()   
            

        self.serial_data_timeout=250
        self.reset_gpio=LED(17)
        self.clutch_gpio=LED(13)
        self.clutch_on_time=0
        #self.pwm_clutch = HardwarePWM(1, hz=PWM_FREQENCY)
        #self.pwm_clutch.start(0)
        self.clutch_delay=CLUTCH_DELAY

        self.enable_gpio=LED(22)
        self.direction_gpio=LED(26)
        
        self.error1_gpio = Button(23)
        self.error2_gpio = Button(24)
        self.pwm_h_bridge = HardwarePWM(0, hz=PWM_FREQENCY)
        self.pwm_h_bridge.start(0)
        #self.error1_gpio.when_pressed(self.hb_error(1))
        #self.error2_gpio.when_pressed(self.hb_error(2))

        
        #ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_128S # 64 samples
        #ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_128S# 64 samples
        #  change voltage range to 16V
        #ina219.bus_voltage_range = BusVoltageRange.RANGE_16V
        
        
        #ads.gain = 2/3    #nur für rohwerte, kein einfluss auf voltage        
        ads.mode = 0x0000   #CONTINUOUS mode reading (ONLY 1 CHANNEL)
        
        self.eeprom_read_addr = 0;
        self.eeprom_read_end = 0;

        self.rudder_sense=1
        self.max_current_value = 0
        self.voltage = self.current = False
        self.flags = 0
        self.low_current=1
        self.pollcount=0
        self.lastpos=1000
        self.params_set=0
        self.timeout=0
        #  change configuration to use 32 samples averaging for both bus voltage and shunt voltage

        self.command_value=0
        
        #params section
        self.max_current = 0
        self.max_controller_temp = 0
        self.max_motor_temp = 0
        self.rudder_min = 0
        self.rudder_max = 65535
        self.max_slew_speed = 0
        self.max_slew_slow = 0
        self.clutch_pwm = 0
        self.currentOffset=2.46207513657033        
        self.lastpositiontime=time.monotonic()

            # 1. RESET auf HIGH setzen GPIO17 PIN11
        self.reset_gpio.off()
            
            # 2. CLUTCH aus: GPIO13 auf low setzen GPIO13 PIN 33
        #self.pwm_clutch.change_duty_cycle(0)
        self.clutch_gpio.off()
        self.clutch_on_time=0
        #print('PWM Clutch auf ',0)
            # 3. ENABLE LED auf Masse legen GPIO22 PIN 15
        self.enable_gpio.on()
#// we need these to be atomic for 16 bytes
        self.rudder_sense=True        
        self.oldtime=time.monotonic()
        self.out_sync_b=0
        self.out_sync_pos=0
        self.serialin=0
        self.sync_b=0
        self.in_bytes=[0,0,0,0]
        self.crcbytes=[0,0,0]
        my_self=self
        #self.sendbuf=[]
        self.lastvalue=0xff
        self.lastaddress=255;
        
        self.xlastvalue=0xff
        self.xlastaddress=255;

        self.t_old=0
        tltl.start()
        #tltl.start()

    def eeprom_read_word(self, addr):
        #print('RaspiMotor eeprom_read_word(',addr,')')
        
        self.myeeprom.eeprom_read_word(addr);
        addr=-addr
        return(addr)
    
    def eeprom_read_16(self, address):
        #print('RaspiMotor eeprom_read_16(',address,') not fully implemented ')
        #// ensure atomic update of 16 bits with 3 banks
        #uint16_t v[3];
        #eeprom_read_16 on 3 locations??')
        v = [1,2,3]
        for  i in range(len(v)) :
            addr = i*256 + address;
            v[i] = self.myeeprom.eeprom_read_word(addr);
            #//eeprom_read_byte((unsigned char *)addr) | eeprom_read_byte((unsigned char *)addr+1)<<8;
        if(v[1] == v[2]):
            return v[2];

        return v[0];

    def eeprom_update_16(self, address, value):
        #// write in 3 locations
        #print('RaspiMotor eeprom_update_16 not fully implemented ')
        for  i in range(3) :
            addr = i*256 + address;
            self.myeeprom.eeprom_update_word(addr, value);

    def eeprom_read_8(self, address):
        #print('RaspiMotor eeprom_read_8(',address,') not fully implemented ')
        
        #self.lastaddress=255;
        value=0xff
        if(address & 1):    # { // odd
            if(address == self.lastaddress+1):
                value = self.lastvalue;
                return 1,value;
            else:
                return 0,value;
        else:    #// even
            v =self.eeprom_read_16(address); #self.eeprom_read_16(address);
            value = v&0xff;
            self.lastvalue = v >> 8;
            self.lastaddress = address;
            return 1,value;#return 1,value;

    def eeprom_update_8(self, address, value):
        #print('RaspiMotor eeprom_update_8 not fully implemented ')
        if(address & 1):    # { // odd
            if(address == self.xlastaddress+1):
                self.eeprom_update_16(self.xlastaddress, self.xlastvalue | value<<8);
        else:
            self.xlastvalue = value;
            self.xlastaddress = address;
    


    def PT_1funk(self, f_grenz, t_abtast, oldvalue, newvalue):
        #const t_abtast= globalStore.getData(keys.properties.positionQueryTimeout)/1000 //[ms->s]
        T = 1 / (2*math.pi*f_grenz)
        tau = 1 / ((T / t_abtast) + 1)
        return(oldvalue + tau * (newvalue - oldvalue))

    def process_packet(self):
        self.flags |= ServoFlags.SYNC;
        value = self.in_bytes[1] | self.in_bytes[2]<<8;
        code=self.in_bytes[0]
        if code==199:
            test=99

        #    Testen wie lange clutch engaged ist
        tt=time.monotonic()-self.clutch_on_time if self.clutch_on_time>0 else 0
        if tt > self.clutch_delay:
            pass
             #print('PWM Clutch auf ',50)
             #print('PWM Clutch auf ',self.clutch_pwm)
             #self.pwm_clutch.change_duty_cycle(50)
        #print('Clutch on: ',ontime)
        
        if code == CommandCodes.REPROGRAM_CODE:
            pass
            #print('RaspiMotor REPROGRAM not implemented ')
        elif code == CommandCodes.RESET_CODE:
            #print('RaspberryServo reset ')
            self.flags &= ~ServoFlags.OVERCURRENT_FAULT
        elif code == CommandCodes.COMMAND_CODE:
            #self.flags = 1
            if(self.serialin < 12):
                self.serialin+=4; #// output at input rate

            self.timeout = 0;
            if(value > 2000):
                #// unused range, invalid!!!
                #// ignored
                pass
            elif(self.flags & (ServoFlags.OVERTEMP_FAULT | ServoFlags.OVERCURRENT_FAULT | ServoFlags.BADVOLTAGE_FAULT)):
                #print('RaspberryServo raw command ERROR1')
                #// no command because of overtemp or overcurrent or badvoltage
                pass
            elif((self.flags & (ServoFlags.PORT_PIN_FAULT | ServoFlags.MAX_RUDDER_FAULT)) and value > 1000):
                #print('RaspberryServo raw command ERROR2')
                self.stop();
            
            # no forward command if port fault
            elif((self.flags & (ServoFlags.STARBOARD_PIN_FAULT | ServoFlags.MIN_RUDDER_FAULT)) and value < 1000):
                #print('RaspberryServo raw command ERROR3')
                self.stop();
                # no starboard command if port fault
            else:
                self.command_value = value;
                self.engage();
                self.update_command()

        elif code == CommandCodes.MAX_CURRENT_CODE:  #// current in units of 10mA
            max_max_current = 2000# if self.lowcurrent else 5000;
            if(value > max_max_current): #// maximum is 20 or 50 amps
                value = max_max_current;
            self.max_current = value;

        elif code == CommandCodes.MAX_CONTROLLER_TEMP_CODE:
            if(value > 10000): #// maximum is 100C
                value = 10000;
            self.max_controller_temp = value;
            
        elif code == CommandCodes.MAX_MOTOR_TEMP_CODE:
            if(value > 10000): #// maximum is 100C
                value = 10000;
            self.max_motor_temp = value;
            
        elif code == CommandCodes.RUDDER_MIN_CODE:
            self.rudder_min = value;
            
        elif code == CommandCodes.RUDDER_MAX_CODE:
            self.rudder_max = value;
            
        elif code == CommandCodes.DISENGAGE_CODE:
            if(self.serialin < 12):
                self.serialin+=4; #// output at input rate
            self.disengage();
            
        elif code == CommandCodes.MAX_SLEW_CODE:
            #print('RaspiMotor MAX_SLEW_CODE: ',value)
            self.max_slew_speed = (value&0xff)
            self.max_slew_slow = ((value>>8)&0xff)  #in_bytes[2];
            #// if set at the end of range (up to 255)  no slew limit
            if(self.max_slew_speed > 250):
                self.max_slew_speed = 250
                if(self.max_slew_slow > 250):
                    self.max_slew_slow = 250;
                    #// must have some slew
                    if(self.max_slew_speed < 1):
                        self.max_slew_speed = 1;
                    if(self.max_slew_slow < 1):
                        self.max_slew_slow = 1;
        
        elif code == CommandCodes.EEPROM_READ_CODE:
            #print('RaspiMotor EEPROM_READ_CODE not implemented ')
            #if(self.eeprom_read_addr == self.eeprom_read_end):
            self.eeprom_read_addr = (value&0xff)     #in_bytes[1];
            self.eeprom_read_end = ((value>>8)&0xff) #in_bytes[2];
            #print('RaspiMotor EEPROM_READ_CODE value:',hex(value),'addr:',hex(value&0xff),'end:',hex( (value>>8)&0xff))
            #print('RaspiMotor EEPROM_READ_CODE value:',hex(value),'addr:',self.eeprom_read_addr,'end:',self.eeprom_read_end)
        elif code == CommandCodes.EEPROM_WRITE_CODE:
            #print('RaspiMotor EEPROM_WRITE_CODE not implemented ')
            self.eeprom_update_8((value&0xff), ((value>>8)&0xff)) #in_bytes[1],in_bytes[2]
        elif code == CommandCodes.CLUTCH_PWM_CODE:
            #print('RaspiMotor CLUTCH_PWM_CODE not implemented ')
            pwm = value #??? in_bytes[1];
            if(pwm < 30):
                pwm = 30;
            elif(pwm > 250):
                pwm = 255;
            self.clutch_pwm = value
        else: 
            print('RaspiMotor unknown CODE: ',code,' in process_packet')


           
    def position(self,value):
        #print(time.monotonic(),'Raspi_Motor position value:',value,'HB-command:',abs(value - 1000)*0.1)
        #self.lastpos = value;
        #OCR1A = abs((int)value - 1000) * 16 / DIV_CLOCK;
        self.pwm_h_bridge.change_duty_cycle( abs(value - 1000)*0.1)
        print('Set Hbridge duty-cycle', abs(value - 1000)*100)
        # Bereich 0 - 1000
        #print('HOWTO H_BRIDGE_PWM(',(int(value) - 1000)/10,'%) @ ', time.monotonic(),' f=',1/(time.monotonic()-self.lastpositiontime)) # abs!!!!
        if(value > 1040):
            self.direction_gpio.off()##print('H_BRIDGE_DIRECTION_off')
            pass
        elif(value < 960):
            self.direction_gpio.on()##print('H_BRIDGE_DIRECTION_off')
            ##print('H_BRIDGE_DIRECTION_on')
            pass
        else:   # nicht nötig
            self.direction_gpio.off()
            ##print('H_BRIDGE_DIRECTION_off')
            pass

    def update_command(self):
#        print('RaspiMotor update_command self.command_value:',self.command_value)
        speed_rate = self.max_slew_speed;
        #// value of 20 is 100% in 1 second full range at 50hz (20*50=1000) --> 50 = 100% in 1 sec at 20Hz
        slow_rate = self.max_slew_slow;

        freq=1/(time.monotonic()-self.lastpositiontime)
        mul=50/freq
        speed_rate *= mul
        slow_rate *= mul

#        print('Raspi_Motor update_command lastpos:',self.lastpos,'commandpos:',self.command_value)
        cur_value = self.lastpos;
        diff = self.command_value - cur_value;
        #print('RaspberryServo update_command diff:',diff)
#        print('HOWTO  diff ',self.command_value,' - ',cur_value,' = ',diff)

        #// limit motor speed change to stay within speed and slow slew rates
        if(diff > 0):
            if(cur_value < 1000):
                if(diff > slow_rate):
                    diff = slow_rate;
            else:
                if(diff > speed_rate):
                    diff = speed_rate;
        else:
            if(cur_value > 1000):
                if(diff < -slow_rate):
                    diff = -slow_rate;
            else:
                if(diff < -speed_rate):
                    diff = -speed_rate;

        #print('Raspi_Motor call position lastpos:',self.lastpos, 'commanded:',self.command_value,'newpos: ', cur_value+diff)
        self.position(cur_value + diff);
        self.lastpositiontime=time.monotonic()
        self.lastpos=cur_value + diff
    
    def hb_error(self,no):
        print('H-Bridge Error ', no)

    def engage(self):
        #if self.flags & ServoFlags.ENGAGED:
         #   return#; // already engaged
        #print('RaspberryServo engage')
        
            # 1. RESET auf HIGH setzen GPIO17 PIN11
        self.reset_gpio.on()
        #print('reset on')
            
            # 2. CLUTCH auf HIGH setzen GPIO13 PIN 33
        self.clutch_gpio.on()
        #self.pwm_clutch.change_duty_cycle(100)
        #print('PWM Clutch auf ',100)
        self.clutch_on_time=time.monotonic()
        #print('clutch on')
            # 3. ENABLE LED auf !!Masse legen GPIO22 PIN 15
        self.enable_gpio.off()
        #print('enable off')

            # 4. Fehlerleitungen PULLUP GPIO23,24 PIN 16,18 -> Jumper J5 auftrennen, dann bekommen die LEDs Spannung von den GPIOs
            #     Pins testen: sollten dann LOW haben wenn alles ok
        #e1.when_pressed(self.hb_error(1))
        #e2.when_pressed(self.hb_error(2))
        self.position(1000);
        #digitalWrite(clutch_pin, HIGH); // clutch
        #clutch_start_time = 20;
        self.flags |= ServoFlags.ENGAGED
             
      
    def disengage(self):
        self.stop();
        if self.flags & ServoFlags.ENGAGED:
            self.flags &= ~ServoFlags.ENGAGED
        timeout = 30; #// detach in about 62ms
        self.pwm_h_bridge.change_duty_cycle(0)

        #clutch_start_time = 0;
        #digitalWrite(clutch_pin, LOW); // clutch
                    # 1. RESET auf HIGH setzen GPIO17 PIN11
        self.reset_gpio.off()
            
            # 2. CLUTCH auf HIGH setzen GPIO13 PIN 33
        #self.pwm_clutch.change_duty_cycle(0)
        self.clutch_gpio.off()
        #print('PWM Clutch auf ',0)
        self.clutch_on_time=0
        
            # 3. ENABLE LED auf Masse legen GPIO22 PIN 15
        self.enable_gpio.on()
        


    def initialize(self): # not used???
        ##print('RaspberryServo initialize')
        cnt = 0
        data = False
        while self.flags & ServoFlags.OVERCURRENT or \
          not self.flags & ServoFlags.SYNC:
            self.stop()
            data = True

            time.sleep(.001)
            cnt+=1
            if cnt == 400 and not data:
                return False
            if cnt == 1000:
                return False
        return True

    
        
    def stop(self):
        #print('RaspiMotor stop')
        self.position(1000) #// 1000 is stopped
        self.command_value = 1000


    def fault(self):
        ##print('RaspberryServo fault: ',self.flags & ServoFlags.OVERCURRENT_FAULT)
        return self.flags & ServoFlags.OVERCURRENT_FAULT

    def max_current(self, value):
        ##print('RaspberryServo max_current ',value)
        self.max_current_value = min(10, value)


    def stop_port(self):
        #print('RaspiMotor stop_port')

        if(self.lastpos > 1000):
            self.stop();

    def stop_starboard(self):
        #print('RaspiMotor stop_starboard')
        if(self.lastpos < 1000):
            self.stop();
    
    
    def TakeRudder(self,p):
    #    // 16 bit value for rudder
        #print('RASPIServo TakeRudder TBD')
        #return TakeADC(ServoTelemetry.RUDDER, p) * 4;
        v= AnalogIn(ads, ADS.P0).voltage /5*65536   # ADC 16bit 0-5V
        #v= Timeloop_125

        t_abtast=(time.monotonic()-self.oldtime)
        #print(time.monotonic(),' RASPIMotor TakeRudder:',v,' T_ABTAST:',t_abtast)
        self.v=self.PT_1funk(0.5, t_abtast, self.v, v)
        v=self.v
        self.oldtime = time.monotonic()       
        #print(time.monotonic(),' RASPIMotor TakeRudder:',v,' T_ABTAST:',t_abtast)
        if(self.rudder_sense): 
            #// if not positive, then rudder feedback has negative gain (reversed)
            pos = self.rudder_min < self.rudder_max;    
            #print('RASPIMotor TakeRudder min,max:',self.rudder_min,self.rudder_max)
            #print('RASPIServo v,rudder_sense',v,self.rudder_sense)
            if((pos and v < self.rudder_min) or (not pos and v > self.rudder_min)):
                self.stop_starboard()
                self.flags |= ServoFlags.MIN_RUDDER_FAULT
            else:
                 self.flags &= ~ServoFlags.MIN_RUDDER_FAULT;
            if((pos and v > self.rudder_max) or (not pos and v < self.rudder_max)):
                self.stop_port();
                self.flags |= ServoFlags.MAX_RUDDER_FAULT;
            else:
                self.flags &= ~ServoFlags.MAX_RUDDER_FAULT;
            if(v < 1024+1024 or v > 65472- 1024):
                self.rudder_sense = 0;  # FEHLER
        elif(v > 1024+1536 and v < 65472 - 1536):
            self.rudder_sense = 1;
            #print('RASPIServo v,rudder_sense',v,rudder_sense)
            self.flags &= ~(ServoFlags.MIN_RUDDER_FAULT | ServoFlags.MAX_RUDDER_FAULT);
        return(v) 
    def TakeAmps(self,p):
        #print('RASPIServo TakeAmps TBD')
        #v = TakeADC(ServoTelemetry.CURRENT, p);
        #return(-ina219.current)*2
        
        
        v= AnalogIn(ads, ADS.P1).voltage 
        v_val=AnalogIn(ads, ADS.P1).value
        if not (self.flags & ServoFlags.ENGAGED):
            self.currentOffset = v -0.01*0.16 # -LL-current  0.160A * 0.01V/A

        v=v-self.currentOffset
        v=v/0.01      # 0.01 V/A
        return v*100
    def TakeVolts(self,p):
        #print('RASPIServo TakeVolts TBD')
        #// voltage in 10mV increments 1.1ref, 560 and 10k resistors
        #v = TakeADC(VOLTAGE, p);
        #return (ina219.bus_voltage+ina219.shunt_voltage)*100
        #return (-ina219.shunt_voltage)*1000
        v= AnalogIn(ads, ADS.P2).voltage 
        return v*10*100 # spannungsteiler 10k/1k

    def read_sensor(self, path):
        value = "999999"
        try:
            f = open(path, "r")
            line = f.readline()
            if re.match(r"([0-9a-f]{2} ){9}: crc=[0-9a-f]{2} YES", line):
                line = f.readline()
                m = re.match(r"([0-9a-f]{2} ){9}t=([+-]?[0-9]+)", line)
                if m:
                    value = float(m.group(2)) / 10.0
            f.close()
        except :
            print ("Error reading")
        return value


    def TakeTemp(self, device):
        #global Timeloop_125
        #print('RASPIServo TakeTemp TBD ', device, TimeloopRetval)
        if(device == ServoTelemetry.CONTROLLER_TEMP):
            #path='/sys/bus/w1/devices/28-00000d3c7f07/w1_slave'
            #print('1-WIRE:',self.read_sensor(path))    
            #return(self.read_sensor(path))
            #return(TimeloopRetval)
            v= AnalogIn(ads, ADS.P3).voltage #1.5V->150°C
            return v*100 #1.5V->150°C
        elif(device == ServoTelemetry.MOTOR_TEMP):
            cpu = CPUTemperature()
            return(cpu.temperature*100)

    def serial_read(self,buffer):
            count = 1
            #if len(buffer>0):
            a=buffer[0]
            buffer.pop(0)
            return a
    def serial_write2(self,buffer,data,l):
        for i in range(l):
            a=len(buffer)
            if(l == 1):
                buffer.append(data)
            else:
                buffer.append(data[i])
        return i
        
            
    #@tltl.job(interval=timedelta(seconds=10), this='that', that='this', other=1)
    #@tltl.job(interval=timedelta(seconds=0.5),args=69)
    def loop2(self,loop):
        i=0
        while(True):
            #print("loop",i)
            i+=1
            time.sleep(0.5)
            pass
        #self=my_self
        #print('Motor process', os.getpid())
        #if not RTIMU:
         #   while True:
          #      time.sleep(10) # do nothing

        #if os.system('sudo chrt -pf 2 %d 2>&1 > /dev/null' % os.getpid()):
         #   print(_('warning, failed to make Motor process realtime'))
        #else:
         #   print(_('made Motor process realtime'))
    
    @tltl.job(interval=timedelta(seconds=0.01))
    def loop():
        global xyz
        self=xyz
        #self.setup()
        if(True):
            t_now=time.monotonic()
            dt=(t_now-self.t_old)
            if dt>0:
                print("Motor-Frequenz: ",1/dt)
            #print("sendbuf", len(self.sendbuf),"receivebuf", len(self.receivebuf))
            self.t_old = t_now
            #print("T: ",time.monotonic())
            i=0
            value=-1
            rudder_value = self.TakeRudder(0)
            
            #Serial einlesen
            in_buf=[]

            #// serial input
            while(len(self.receivebuf)):
                  c = self.serial_read(self.receivebuf);#   uint8_t c = Serial.read();
                  #self.serialin+=c
                  self.serial_data_timeout = 0;
                  self.flags = abs(self.flags)
                  if(self.sync_b < 3):
                      self.in_bytes[self.sync_b] = c;
                      self.sync_b+=1;
                  else:
                      d=crc8(self.in_bytes, 3)
                      if(c == d):    
                          if(self.in_sync_count >= 2):   # { // if crc matches, we have a valid packet
                              #print("process packet:",self.in_bytes)
                              self.process_packet();
                          else:
                              self.in_sync_count+=1;
                          self.sync_b = 0;
                          self.flags &= ~ServoFlags.INVALID;
                      else: 
                          #// invalid packet
                          self.flags &= ~ServoFlags.SYNC;
                          self.in_sync_count = 0;
                          self.in_bytes[0] = self.in_bytes[1]; #// shift input 1 byte
                          self.in_bytes[1] = self.in_bytes[2];
                          self.in_bytes[2] = c;
                          self.flags |= ServoFlags.INVALID;
                      break

            code = results.UNKNOWN_CODE;

            if self.out_sync_b==0: 
                 #   break;
                code = results.UNKNOWN_CODE;
                #//  flags C R V C R ct C R mt flags  C  R  V  C  R EE  C  R mct flags  C  R  V  C  R  EE  C  R rr flags  C  R  V  C  R EE  C  R cc  C  R vc
                #//  0     1 2 3 4 5  6 7 8  9    10 11 12 13 14 15 16 17 18  19    20 21 22 23 24 25  26 27 28 29    30 31 32 33 34 35 36 37 38 39 40 41 42
                #switch(out_sync_pos++) {
                self.out_sync_pos+=1
                #print("self.out_sync_pos",self.out_sync_pos)
                
                i=self.out_sync_pos
                numlist=[0,10,20,30]
                if(i in numlist):
                    #self.flags=0
                    if(not self.low_current):
                        self.flags |= ServoFlags.CURRENT_RANGE;
#                    self.flags &= ~ServoFlags.REBOOTED;
                    #self.flags=0
                    self.flags |= ServoFlags.SYNC
                    #self.flags |= ServoFlags.ENGAGED#kds
                    value = abs(self.flags)
                    code = results.FLAGS_CODE;
                    #print("RaspiMotor ret.code:",ret.code,' value:',ret.value)
                numlist=[1,4,7,11,14,17,21,24,27,31,34,37,40]
                if(i in numlist):
                    value = self.current=self.TakeAmps(0);
                    code = results.CURRENT_CODE
                numlist=[2,  5,  8,  12,  15,  18,  22,  25,  28,  32,  35,  38,  41]
                if(i in numlist):
                    if(self.rudder_sense == 0):
                        value = self.rudder = 65535   #; // indicate invalid rudder measurement
                    else:
                        value = self.rudder = rudder_value  #self.TakeRudder(0);
                    code = results.RUDDER_SENSE_CODE
                    #print("RaspiMotor ret.code:",results.RUDDER_SENSE_CODE,' value:',value)
                    
                numlist=[ 3,  13,  23,  33]
                if(i in numlist):
                    value = self.voltage = self.TakeVolts(0);
                    code = results.VOLTAGE_CODE
                numlist=[6]
                if(i in numlist):
                        value = self.controller_temp=self.TakeTemp(ServoTelemetry.CONTROLLER_TEMP);
                        code = results.CONTROLLER_TEMP_CODE
                numlist=[9]
                if(i in numlist):
                        value = self.motor_temp = self.TakeTemp(ServoTelemetry.MOTOR_TEMP)
                        code = results.MOTOR_TEMP_CODE
                numlist=[ 16,  26,  36]            
                if(i in numlist):
                    #print('RASPI_MOTOR EEPROM_VALUE_CODE retval 16,26,36 read_addr:',self.eeprom_read_addr,'read_end:', self.eeprom_read_end)
                    if(self.eeprom_read_addr < self.eeprom_read_end):
                        ret,retvalue=self.eeprom_read_8(self.eeprom_read_addr)
                        #print('RASPI_MOTOR EEPROM_VALUE_CODE2 ret:',ret,' retvalue:',retvalue)
                        if(ret): 
                            v = retvalue << 8 | self.eeprom_read_addr;
                            self.eeprom_read_addr+=1;
                            code = results.EEPROM_VALUE_CODE;
                            value = v
                            self.out_sync_pos-=1#; // fast eeprom read
                            #print('RASPI_MOTOR EEPROM_VALUE_CODE retval 16,26,36 returns:',hex(v))
                        #self.eeprom_read_addr+=1; #// skip for now
                    else:
                        return
                    
                self.crcbytes[0] = int(code)&0xFF;
                self.crcbytes[1] = int(value)&0xFF;
                self.crcbytes[2] = int(value)>>8 &0xFF;
                atest=self.crcbytes[2]<<8+self.crcbytes[1]
                if atest != value:
                    atest=-1
                    
                    #// write next
                i+=1
                print("self.out_sync_pos",self.out_sync_pos)
                if self.out_sync_pos > 52:
                    self.out_sync_pos=0
                self.out_sync_b+=1
            elif self.out_sync_b==3 :
                        self.serial_write2(self.sendbuf, self.crcbytes[0],1)
                        self.serial_write2(self.sendbuf, self.crcbytes[1],1)
                        self.serial_write2(self.sendbuf, self.crcbytes[2],1)
                        #// write crc of sync byte plus bytes transmitted
                        a=crc8(self.crcbytes, 3)
                        self.serial_write2(self.sendbuf, crc8(self.crcbytes, 3),1);
                        self.out_sync_b = 0;
            else:
                self.out_sync_b+=1
                        #// match output rate to input rate
                        #if(self.serialin < 4):
        
                        #if True:    #t > 0 and t < period:
                        #else:
                         #   print(_('Motor process failed to keep time'), dt, t0, t1, t2, t3)
            
