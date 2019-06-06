import pyb

class CRSBotBrainMD:
    ''' This class implements the driver for the Continuous Rotation
    Servo (CRS) by BotBrain. This motor will be used to actuate our
    trigger mechanism on our 405 project.'''
    
    
    def __init__ (self, signal_pin, timer):
        ''' Creates a motor driver by initializing the signal pin 
        used to transmit pulse widths to the servo motor.'''
        
        self.signal_pin = signal_pin
        
        self.pwm_ch = timer.channel (1, pyb.Timer.PWM, pin = signal_pin)
        
        
    def set_duty_cycle (self, level):
        ''' This method sets the duty cycle to be sent
        to the motor to the given level, which will affect
        the anglular displacement of the servo.
        @param level A signed integer holding the duty
            cycle of the voltage sent to the motor '''
        
        self.pwm_ch.pulse_width_percent(level)
        
            
        

     
        
