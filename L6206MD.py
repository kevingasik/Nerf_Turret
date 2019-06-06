import pyb

class L6206MD:
    ''' This class implements the driver for L6206 Motor Driver,
    which can be found on the ME405 board. This driver will be
    used to power DC motors to control span and tilt on our
    ME 405 term project.'''
    
    
    
    def __init__ (self, enable_pin, pinA, pinB, timer):
        ''' Creates a motor driver by initializing GPIO
        pins and turning the motor off for safety. '''
        
        self.enable_pin = enable_pin
        self.enable_pin.low()
        
        self.ch1 = timer.channel (1, pyb.Timer.PWM, pin=pinA)
        self.ch2 = timer.channel (2, pyb.Timer.PWM, pin=pinB)
        
        
    def set_duty_cycle (self, level):
        ''' This method sets the duty cycle to be sent
        to the motor to the given level. Positive values
        cause torque in one direction, negative values
        in the opposite direction.
        @param level A signed integer holding the duty
            cycle of the voltage sent to the motor '''
        
        if level < 0:
            self.ch1.pulse_width_percent(-level)
            self.ch2.pulse_width_percent(0)
        else:
            self.ch1.pulse_width_percent(0)
            self.ch2.pulse_width_percent(level)
            
        self.enable_pin.high()
        

     
        
