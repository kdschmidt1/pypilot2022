
import re, os
from pypilot.arduino_servo.raspberry_servo_python2 import *
from servo import *
from pypilot.arduino_servo.myeeprom import *


import board
#from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from timeloop import Timeloop
from datetime import timedelta
import time

#tltl = Timeloop()
#global TimeloopRetval
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



class RaspberryMotor:
    def __init__(self):
        self.myeeprom = myeeprom()
        self.myeeprom.eeprom_read_byte(5)
        self.v=0
        self.oldtime = time.monotonic()   
            

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
#        tltl.start()


    def eeprom_read_word(self, addr):
        print('RaspiMotor eeprom_read_word(',addr,')')
        
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
        print('RaspiMotor eeprom_update_16 not fully implemented ')
        for  i in range(3) :
            addr = i*256 + address;
            self.myeeprom.eeprom_update_word(addr, value);

    def eeprom_read_8(self, address):
        #print('RaspiMotor eeprom_read_8(',address,') not fully implemented ')
        
        lastvalue=0xff
        lastaddress=255;
        value=0xff
        #if(address & 1):    # { // odd
         #   if(address == lastaddress+1):
          #      value = lastvalue;
           #     return 1,value;
            #else:
             #   return 0,value;
        #else:    #// even
        v =self.myeeprom.eeprom_read_byte(address); #self.eeprom_read_16(address);
        value = v&0xff;
        lastvalue = v >> 8;
        lastaddress = address;
        return 1,value;

    def eeprom_update_8(self, address, value):
        #print('RaspiMotor eeprom_update_8 not fully implemented ')
        self.myeeprom.eeprom_update_byte(address, value);
        return
        lastvalue=0xff
        lastaddress=255;
        if(address & 1):    # { // odd
            if(address == lastaddress+1):
                eeprom_update_16(lastaddress, lastvalue | value<<8);
        else:
            lastvalue = value;
            lastaddress = address;


    def PT_1funk(self, f_grenz, t_abtast, oldvalue, newvalue):
        #const t_abtast= globalStore.getData(keys.properties.positionQueryTimeout)/1000 //[ms->s]
        T = 1 / (2*math.pi*f_grenz)
        tau = 1 / ((T / t_abtast) + 1)
        return(oldvalue + tau * (newvalue - oldvalue))
    def process_packet(self, code,value):
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
        #self.pwm_h_bridge.change_duty_cycle( abs(value - 1000)*0.1)
        #print('Set Hbridge duty-cycle', abs(value - 1000)*100)
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
#        print('RaspberryServo update_command diff:',diff)
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
        if self.flags & ServoFlags.ENGAGED:
            return#; // already engaged
        print('RaspberryServo engage')
        
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
        if(self.flags or ServoFlags.ENGAGED):
            self.flags &= ~ServoFlags.ENGAGED;
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
        print('RaspiMotor stop_port')

        if(self.lastpos > 1000):
            self.stop();

    def stop_starboard(self):
        print('RaspiMotor stop_starboard')
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
        offset=2.46207513657033
        
        v= AnalogIn(ads, ADS.P1).voltage 
        v_val=AnalogIn(ads, ADS.P1).value
        
        v=v-offset #1.5V->150°C
        v=v/0.028      
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

    def ret_val(self, i):
        value=-1
        rudder_value = self.TakeRudder(0)
        
        #print('ret_val: ',time.monotonic())
        code = results.UNKNOWN_CODE;
        numlist=[0,10,20,30]
        if(i in numlist):
            if(not self.low_current):
                self.flags |= ServoFlags.CURRENT_RANGE;
            self.flags &= ~ServoFlags.REBOOTED;
            self.flags |= ServoFlags.SYNC
            value = self.flags
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
            code = results.EEPROM_VALUE_CODE;
            if(self.eeprom_read_addr != self.eeprom_read_end):
                ret,retvalue=self.eeprom_read_8(self.eeprom_read_addr)
                #print('RASPI_MOTOR EEPROM_VALUE_CODE2 ret:',ret,' retvalue:',retvalue)
                if(ret): 
                    v = retvalue << 8 | self.eeprom_read_addr;
                    self.eeprom_read_addr+=1;
                    if(self.eeprom_read_addr>20):
                        self.eeprom_read_addr=0
                    code = results.EEPROM_VALUE_CODE;
                    value = v
                    #out_sync_pos-=1#; // fast eeprom read
                    #print('RASPI_MOTOR EEPROM_VALUE_CODE retval 16,26,36 returns:',hex(v))
                    return(code,value)
            self.eeprom_read_addr+=1; #// skip for now
            if(self.eeprom_read_addr>20):
                self.eeprom_read_addr=0
            if value==-1:
                test=True
        try:
            return(code,value)
        except:
            pass
