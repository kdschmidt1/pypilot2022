#KDS1
#for i,v in self.local.items():
 #   print(i,v)


# list of class variables:
#[attr for attr in dir(self.arduino_servo_data()) if not callable(getattr(self.arduino_servo_data(),attr)) and not attr.startswith("__")]


#// This structure is stored in self memory
import smbus
import time

class EEPROM24Cxx():
    # Init I2C
    # abhaengig von der Jumperstellung kann das EEPROM auf den
    # Adressen 0x50 bis 0x57 liegen - ggf. mit i2cdetect ermitteln
    def __init__(self, twi=1, addr=0x50):
        self._bus = smbus.SMBus(twi)
        self._addr = addr

    def read_byte(self,addr):
        # Lesen eines Bytes aus dem EEPROM
       #print("Read_Byte von Adresse ",addr)
       self._bus.write_byte_data(self._addr, addr//256, addr%256)
       return self._bus.read_byte(self._addr)

    def write_byte(self, addr, byte):
        print("Write_Byte ",byte," an Adresse ",addr)
        # Schreiben eines Bytes in das EEPROM
        data = [addr%256, byte]
        self._bus.write_i2c_block_data(self._addr, addr//256, data)
        #self._bus.write_byte(self._addr,0x99)
        time.sleep(0.015) # data sheet says 10 msec max





arduino_servo_data={# /*__attribute__(("packed"))*/ {
    'max_current':0, 
    'max_controller_temp':0, 
    'max_motor_temp':0,
    'rudder_range':0,
    'rudder_offset':0,
    'rudder_scale':0,
    'rudder_nonlinearity':0,
    'max_slew_speed':0, 
    'max_slew_slow':0,
    'current_factor':0, 
    'voltage_factor':0,
    'current_offset':0, 
    'voltage_offset':0,
    'min_speed':0, 
    'max_speed':0,
    'gain':0,
    'clutch_pwm':0,
    'signature':0,
}
 #// changes if self format changes,
             #          // put at end so it's written last


class myeeprom:
    initial_read=False    
    #verified=[]
    eeprom24cxx = EEPROM24Cxx(1,0x50)
    #local = arduino_servo_data; #// local version of servo data
    def __init__(self):
        self.verified=[]

        self.local = arduino_servo_data; #// local version of servo data
        #private:
        self.arduino=self.local.copy()
        self.local['signature']=9876 
        self.arduino['signature']=-1        
        #self.members = self.eeprom.arduino['members=[attr for attr in dir(eeprom.arduino_servo_data()) if not callable(getattr(eeprom.arduino_servo_data(),attr)) and not attr.startswith("__")]

        self.verified=[ (False)for x in range(2*len(self.local)) ]#wg 2Byte / Eintrag
        #for x in range(len(self.eeprom.arduino)):
            #self.verified.append(False)
    def eeprom_update_byte(self, addr, value):
        #print("eeprom_update_byte", addr, hex(value))
        self.eeprom24cxx.write_byte(addr, value)
    def eeprom_update_word(self, addr, value):
        #print("eeprom_update_word", addr, hex(value))
        self.eeprom24cxx.write_byte(addr, (value&0Xff00)>>8)
        self.eeprom24cxx.write_byte(addr+1, (value&0Xff))
    def eeprom_read_byte(self, addr):
        return self.eeprom24cxx.read_byte(addr)
    def eeprom_read_word(self, addr):
        value=self.eeprom24cxx.read_byte(addr)
        value <<=8
        value+=self.eeprom24cxx.read_byte(addr+1)
        return(value)
    
    
    def need_read(self,end): # -1 == no-need, all verified
    #int is = sizeof verified;
        for i in range(len(self.verified)):
            if(not self.verified[i]):
                end = i;
                while(end < len(self.verified) and (not self.verified[end])):
                    end+=1
                if(end and 1): #// make even
                    end+=1;
                    #print('self-need_read returns ',(i and ~1))
                #print('self-need_read returns ',i and ~1,end)
                return (i&~1,end);# // always even
        #print('self-need_read returns ',-1,end)
        return -1,end
    def need_write(self,):   # sollte die Adresse des veränderten self.value zurückgeben 
        #print('self-need_write')
        if(not self.initial_read): #// don't write before reading
            #print('self-need_write1 returns ',-1)
            return -1;

#    uint8_t *l = (uint8_t*)&self.local, *a = (uint8_t*)&self.arduino;
    #int ls = sizeof self.local;
        k=0
        for i,v in self.local.items():
            if( self.local[i] != self.arduino[i] and self.verified[k]):
                k&=~1; #// always even
                self.verified[k] = 0;    # // read this byte again
                self.verified[k+1] = 0; #// read this byte again
                #print('self-need_write2 returns ',k)
                   # print('self-need_write ',k)
                return k;
            k+=2
      #  print('self-need_write ',-1)
        return -1;

#// return true  only once ever if initial self data is read
    def initial(self):
        ret,end=self.need_read(0)
        if(self.initial_read or ret != -1):
            #print('EEPROMX-self.initial returns False',self.need_read(0))
            return False;
        #print('self-initial set initial_read True')
        self.initial_read = True; #// allow writes now
        #print('self-self.initial_read ',self.initial_read)
        #print('self-initial_read= ',-1)
        #// signature failed, discard this data
        if(self.arduino['signature']!=self.local['signature']):
            print("self.arduino self Signature FAILED!\n");
            return False;

    #uint8_t *a = (uint8_t*)&self.arduino;
    #int ls = sizeof self.arduino;

    #// discard if any byte is 0xff.
    #// This is invalid data and is also the initial self value
        #print('self self.verified ',self.verified)
        #print('Initial self.self.arduino:',self.self.arduino)
        for i,v in self.arduino.items():
            #print('Initial i,v:',i,v)
            if(v == 0xff):
                #print("self SIGNATURE invalid byte %d\n", i);
                return False;
    
        print("self SIGNATURE ok\n");
        return True;
    
    def value(self,addr, val):  # 1byte-Wert
        if(addr >= len(self.verified)):
            return;
        if addr%2:
            #ungerade--> Highbyte
            self.arduino[list(self.arduino)[int(addr/2)]] &=0xFF
            self.arduino[list(self.arduino)[int(addr/2)]] |= val<<8;
        else:
            self.arduino[list(self.arduino)[int(addr/2)]] &=0xFF00
            self.arduino[list(self.arduino)[int(addr/2)]] |= val;
        self.verified[addr] = True;
        #print("self VALUE Verify :",list(self.arduino)[addr],val);

    def get_max_current(self):
        return self.frombase255(self.arduino['max_current'])/100.0;

    def set_max_current(self, max_current):
        self.local['max_current'] = self.tobase255(round(max_current * 100.0));

    def get_max_controller_temp(self):
        return self.frombase255(self.arduino['max_motor_temp'])/100.0;

    def set_max_controller_temp(self, max_controller_temp):
        self.local['max_controller_temp'] = self.tobase255(round(max_controller_temp * 100.0));

    def get_max_motor_temp(self):
        return self.frombase255(self.arduino['max_motor_temp'])/100.0;

    def set_max_motor_temp(self,  max_motor_temp):
        self.local['max_motor_temp'] = self.tobase255(round(max_motor_temp * 100.0));


    def get_rudder_range(self):
        return self.arduino['rudder_range']/2.0;

    def set_rudder_range(self,  rudder_range):

        self.local['rudder_range'] = round(rudder_range * 2); #// from 0 to 120 in 0.5 increments


#// store offset as s9.6 fixed point
    def get_rudder_offset(self):
        return self.frombase255s(self.arduino['rudder_offset'])/64.0;

    def set_rudder_offset(self,  rudder_offset):
        self.local['rudder_offset'] = self.tobase255s( round(rudder_offset * 64));

#// store rudder scale s12.3 fixed point
    def get_rudder_scale(self):
        return self.frombase255s(self.arduino['rudder_scale'])/8.0;

    def set_rudder_scale(self,  rudder_scale):
        self.local['rudder_scale'] = self.tobase255s( round(rudder_scale * 8.0));

#// store nonlinearity s12.3 fixed point
    def get_rudder_nonlinearity(self):
        return self.frombase255s(self.arduino['rudder_nonlinearity'])/8.0;

    def set_rudder_nonlinearity(self,  rudder_nonlinearity):
        self.local['rudder_nonlinearity'] = self.tobase255s( round(rudder_nonlinearity * 8.0));

    def get_max_slew_speed(self):
        return self.arduino['max_slew_speed']/254.0*100.0;

    def set_max_slew_speed(self,  max_slew_speed):
        self.local['max_slew_speed'] = round(max_slew_speed * 254/100);


    def get_max_slew_slow(self):
        return self.arduino['max_slew_slow']/254.0*100.0;

    def set_max_slew_slow(self,  max_slew_slow):
        self.local['max_slew_slow'] = round(max_slew_slow * 254/100);

    def get_current_factor(self):
        return .8 + self.arduino['current_factor'] * .0016;

    def set_current_factor(self,  current_factor):
        self.local['current_factor'] = round((current_factor - .8)/.0016);


    def get_current_offset(self):
        return (self.arduino['current_offset']-127)/100.0;


    def set_current_offset(self,  current_offset):
        self.local['current_offset'] = 127+round(current_offset*100.0);


    def get_voltage_factor(self):
        return .8 + self.arduino['voltage_factor'] * .0016;


    def set_voltage_factor(self,  voltage_factor):
        self.local['voltage_factor'] = round((voltage_factor - .8)/.0016);


    def get_voltage_offset(self):
        return (self.arduino['voltage_offset']-127)/100.0;


    def set_voltage_offset(self,  voltage_offset):
        self.local['voltage_offset'] = 127 + round(voltage_offset*100.0);


    def get_min_speed(self):
        return self.arduino['min_speed']/2.0;

     
    def set_min_speed(self,  min_speed):
        self.local['min_speed'] = round(min_speed*2.0);


    def get_max_speed(self):
        return self.arduino['max_speed']/2.0;
     
    def set_max_speed(self, max_speed):
        self.local['max_speed'] = round(max_speed*2.0);


#// record gain in thousandths
    def get_gain(self):
        return self.frombase255s(self.arduino['gain'])/1000.0;


    def set_gain(self, gain):
        self.local['gain'] = self.tobase255s( round(gain*1000.0));


#// record pwm in percent
    def get_clutch_pwm(self):
        return self.arduino['clutch_pwm']/2.54;


    def set_clutch_pwm(self, pwm):
        self.local['clutch_pwm'] = int(pwm*2.54);

    def tobase255(self,x):
        #// max value is 65024, and 0xff is forbidden in either byte
        h = int(x/255)
        l = int(x%255);

        if(h > 255):
            h = 255; #// invalid

        return (h<<8) | l;

    def frombase255(self,x):
        #// max value is 65024, and 0xff is forbidden in either byte
        h = (x>>8)
        l = x&0xff;
        return h*255 + l;

    def tobase255s(self,x):
        if(x<-32512 or x>32512):
            return 0xffff;
        return self.tobase255(x+32512);

    def frombase255s(self,x):
        z = self.frombase255(x);
        return z  - 32512;

