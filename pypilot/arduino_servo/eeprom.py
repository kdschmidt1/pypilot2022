#for i,v in eeprom.local.items():
 #   print(i,v)


# list of class variables:
#[attr for attr in dir(eeprom.arduino_servo_data()) if not callable(getattr(eeprom.arduino_servo_data(),attr)) and not attr.startswith("__")]


#// This structure is stored in eeprom memory


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
 #// changes if eeprom format changes,
             #          // put at end so it's written last


class eeprom:
    initial_read=False    
    verified=[]
    local = arduino_servo_data; #// local version of servo data
    def __init__(self):
        eeprom.local = arduino_servo_data; #// local version of servo data
        #private:
        eeprom.arduino=eeprom.local.copy()
        eeprom.local['signature']=9876        
        #self.members = self.eeprom.arduino['members=[attr for attr in dir(eeprom.arduino_servo_data()) if not callable(getattr(eeprom.arduino_servo_data(),attr)) and not attr.startswith("__")]

        eeprom.verified=[(False)for x in range(len(eeprom.local))]
        #for x in range(len(self.eeprom.arduino)):
            #self.verified.append(False)
    def need_read(end): # -1 == no-need, all verified
    #int is = sizeof verified;
        for i in range(len(eeprom.verified)):
            if(not eeprom.verified[i]):
                end = i;
                while(end < len(eeprom.verified) and (not eeprom.verified[end])):
                    end+=1
                if(end and 1): #// make even
                    end+=1;
                    #print('EEPROM-need_read returns ',(i and ~1))
                #print('EEPROM-need_read returns ',i and ~1,end)
                return (i&~1,end);# // always even
        #print('EEPROM-need_read returns ',-1,end)
        return -1,end
    def need_write():   # sollte die Adresse des veränderten eeprom.value zurückgeben 
        print('EEPROM-need_write')
        if(not eeprom.initial_read): #// don't write before reading
            #print('EEPROM-need_write1 returns ',-1)
            return -1;

#    uint8_t *l = (uint8_t*)&eeprom.local, *a = (uint8_t*)&eeprom.arduino;
    #int ls = sizeof eeprom.local;
        
        k=0
        for i,v in eeprom.local.items():
            if( eeprom.local[i] != eeprom.arduino[i] and  eeprom.verified[k]):  
                k &= ~1;    # // always even
                eeprom.verified[k] = 0;    # // read this byte again
                eeprom.verified[k+1] = 0; #// read this byte again
                #print('EEPROM-need_write2 returns ',k)
                return k;
            k += 1
        print('EEPROM-need_write ',k)
        return -1;

#// return true  only once ever if initial eeprom data is read
    def initial(self):
        ret,end=eeprom.need_read(0)
        if(eeprom.initial_read or ret != -1):
            #print('EEPROMX-eeprom.initial returns False',eeprom.need_read(0))
            return False;
        #print('EEPROM-initial set initial_read True')
        eeprom.initial_read = True; #// allow writes now
        #print('EEPROM-eeprom.initial_read ',eeprom.initial_read)
        #print('EEPROM-initial_read= ',-1)
        #// signature failed, discard this data
        if(eeprom.eeprom.arduino['signature']!=eeprom.eeprom.local['signature']):
            print("eeprom.arduino EEPROM Signature FAILED!\n");
            return False;

    #uint8_t *a = (uint8_t*)&eeprom.arduino;
    #int ls = sizeof eeprom.arduino;

    #// discard if any byte is 0xff.
    #// This is invalid data and is also the initial eeprom value
        #print('EEPROM self.verified ',eeprom.verified)
        #print('Initial eeprom.eeprom.arduino:',eeprom.eeprom.arduino)
        for i,v in eeprom.eeprom.arduino.items():
            #print('Initial i,v:',i,v)
            if(v == 0xff):
                #print("EEPROM SIGNATURE invalid byte %d\n", i);
                return False;
    
        print("EEPROM SIGNATURE ok\n");
        return True;
    
    def value(addr, val):
        if(addr >= len(eeprom.local)):
            return;
        eeprom.arduino[addr] = val;
        eeprom.verified[addr] = 1;
        print("EEPROM VALUE Verify :",addr);

    def get_max_current():
        return self.frombase255(eeprom.arduino['max_current'])/100.0;

    def set_max_current(max_current):
        eeprom.local['max_current'] = eeprom.tobase255(round(max_current * 100.0));

    def get_max_controller_temp():
        return self.frombase255(eeprom.arduino['max_motor_temp'])/100.0;

    def set_max_controller_temp(max_controller_temp):
        eeprom.local['max_controller_temp'] = eeprom.tobase255(round(max_controller_temp * 100.0));

    def get_max_motor_temp():
        return self.self.frombase255(eeprom.arduino['max_motor_temp'])/100.0;

    def set_max_motor_temp( max_motor_temp):
        eeprom.local['max_motor_temp'] = eeprom.tobase255(round(max_motor_temp * 100.0));


    def get_rudder_range():
        return eeprom.arduino['rudder_range']/2.0;

    def set_rudder_range( rudder_range):

        eeprom.local['rudder_range'] = round(rudder_range * 2); #// from 0 to 120 in 0.5 increments


#// store offset as s9.6 fixed point
    def get_rudder_offset():
        return self.frombase255s(eeprom.arduino['rudder_offset'])/64.0;

    def set_rudder_offset( rudder_offset):
        eeprom.local['rudder_offset'] = eeprom.tobase255s(round(rudder_offset * 64));

#// store rudder scale s12.3 fixed point
    def get_rudder_scale():
        return self.frombase255s(eeprom.arduino['rudder_scale'])/8.0;

    def set_rudder_scale( rudder_scale):
        eeprom.local['rudder_scale'] = eeprom.tobase255s(round(rudder_scale * 8.0));

#// store nonlinearity s12.3 fixed point
    def get_rudder_nonlinearity():
        return self.frombase255s(eeprom.arduino['rudder_nonlinearity'])/8.0;

    def set_rudder_nonlinearity( rudder_nonlinearity):
        eeprom.local['rudder_nonlinearity'] = eeprom.tobase255s(round(rudder_nonlinearity * 8.0));

    def get_max_slew_speed():
        return eeprom.arduino['max_slew_speed']/254.0*100.0;

    def set_max_slew_speed( max_slew_speed):
        eeprom.local['max_slew_speed'] = round(max_slew_speed * 254/100);


    def get_max_slew_slow():
        return eeprom.arduino['max_slew_slow']/254.0*100.0;

    def set_max_slew_slow( max_slew_slow):
        eeprom.local['max_slew_slow'] = round(max_slew_slow * 254/100);

    def get_current_factor():
        return .8 + eeprom.arduino['current_factor'] * .0016;

    def set_current_factor( current_factor):
        eeprom.local['current_factor'] = round((current_factor - .8)/.0016);


    def get_current_offset():
        return (eeprom.arduino['current_offset']-127)/100.0;


    def set_current_offset( current_offset):
        eeprom.local['current_offset'] = 127+round(current_offset*100.0);


    def get_voltage_factor():
        return .8 + eeprom.arduino['voltage_factor'] * .0016;


    def set_voltage_factor( voltage_factor):
        eeprom.local['voltage_factor'] = round((voltage_factor - .8)/.0016);


    def get_voltage_offset():
        return (eeprom.arduino['voltage_offset']-127)/100.0;


    def set_voltage_offset( voltage_offset):
        eeprom.local['voltage_offset'] = 127 + round(voltage_offset*100.0);


    def get_min_speed():
        return eeprom.arduino['min_speed']/2.0;

     
    def set_min_speed( min_speed):
        eeprom.local['min_speed'] = round(min_speed*2.0);


    def get_max_speed():
        return eeprom.arduino['max_speed']/2.0;
     
    def set_max_speed(max_speed):
        eeprom.local['max_speed'] = round(max_speed*2.0);


#// record gain in thousandths
    def get_gain():
        return self.frombase255s(eeprom.arduino['gain'])/1000.0;


    def set_gain(gain):
        eeprom.local['gain'] = eeprom.tobase255s(round(gain*1000.0));


#// record pwm in percent
    def get_clutch_pwm():
        return eeprom.arduino['clutch_pwm']/2.54;


    def set_clutch_pwm(pwm):
        eeprom.local['clutch_pwm'] = pwm*2.54;

    def tobase255(x):
        #// max value is 65024, and 0xff is forbidden in either byte
        h = int(x/255)
        l = int(x%255);

        if(h > 255):
            h = 255; #// invalid

        return (h<<8) | l;

    def frombase255(x):
        #// max value is 65024, and 0xff is forbidden in either byte
        h = (x>>8)
        l = x&0xff;
        return h*255 + l;

    def tobase255s(x):
        if(x<-32512 or x>32512):
            return 0xffff;
        return eeprom.tobase255(x+32512);

    def frombase255s(x):
        z = eeprom.frombase255(x);
        return z  - 32512;

