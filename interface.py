# interface.py
# Kevin Scott, Kevin, Gasik, Caleb Barber

import serial

class Interface:
    ''' Class for performing closed-loop proportional control'''
    
    def __init__(self)

    def serialSend(self, time, position):
        for i in range(len(time)):
            #string_time = str(time[i]) 
            #string_position = str(position[i]) 
            string = '{d:} {d:}'.format(time[i],position[i])
            with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser_port:
                ser_port.write(string)
                a_line = (ser_port.readline().decode('UTF_8')
   

from matplotlib import pyplot   

with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser_port:
    a_line = (ser_port.readline().decode('UTF_8')
              
time = []
position = []

num_strs = a_line.split(',')
try:
    t = float(num_strs[0])
    p = float(num_strs[1])
except:
    pass
else:
    time.append(t)
    position.append(p)