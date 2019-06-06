#Lab1
#Caleb Barber, Kevin Gasik, Kevin Scott

import pyb


class EncoderReader:
    '''Class Encoder Reader uses an internal Timer counter on the nucleo board. The Timer counter uses channels 1 and 2. Class Encoder Reader reads a value from an encoder that is attached to a motor. The Class is modular meaning you need to intialize the pins you want to use and specify the timer that you want to use(You can not use timer chn 5).'''
    

    def __init__ (self, pinA, pinB, timer):
        '''Initializing the constructor for class Encoder Reader. This constuctor takes the three values passed to Encoder reader and intializes the Encoder to be used on the pins specified and the timer channel specified. This function also creates a timer on timer counter 6.'''
        
        
        #set up timer channels
        ## callbackTimer is the timer which causes the read() method to continuously be called at 100Hz.
        self.callbackTimer = pyb.Timer (6, freq=100)
        timer.channel (1, pyb.Timer.ENC_AB, pin=pinA)
        timer.channel (2, pyb.Timer.ENC_AB, pin=pinB)
        ##the current count on the timer specified to the class EncoderReader
        self.current_count = timer.counter()
        ## the current position of the encoder
        self.position = 0
        ## encTimer is the encoderTimer that was specified to class EncoderReader
        self.encTimer = timer
        self.read()
        self.callbackTimer.callback(lambda t: self.read())

    def read(self):
        ''' The read function returns an integer value of the current position and an integer delta. Delta is the difference between current count and last count. The read function handles overflow of the timercounter being used.The timer channel being used is an int 16 type and therefore can be overflowed, this function handles this issue and returns the true encoder position'''
        ## last_count is the last count through the hardware loop of the encoder. It stores the previous count that was stored in current count. 
        self.last_count = self.current_count
        self.current_count = self.encTimer.counter() 
        delta = self.current_count - self.last_count
        if delta > 32767:
            delta = -65535 + delta
        if delta < -32767:
            delta = (65535 + delta)
        self.position = self.position + delta
        
    def get_position(self):
        ''' The get_position function prints out the current encoder position.'''
        self.read()
        #print(self.position)
        return self.position
    
    
    def zero(self):
        '''The Zero function's purpose is to reset the encoder value. The zero function reads the encoder and then sets the current position  to be equal to 0. It also resets the timer Counter that was specified to the class to be equal to the current count. '''
        self.read() 
        self.position = 0
        self.current_count = self.encTimer.counter()
        
        
        

#if (__name__ == '__main__'):
	#enc = EncoderReader()
	#enc.read()
