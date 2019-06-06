import serial
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib import pyplot


ser_port = serial.Serial('COM7',115200, timeout=1)
ser_port.flush()
ser_port.write(b'\x04\n') # make sure to ctrl-D before 

count = 0
state = 0
mylist = []

Running = True

def readSerPort():
    if(ser_port != None):
        if(ser_port.is_open and ser_port.in_waiting):
            data = (ser_port.readline()).decode ('UTF_8')
            print (data)

def PlotData(lines):
    xdata = []
    ydata = []
    for line in lines: 
        num_strs = line.split(',')
        try:
            x = float(num_strs[0])
            y = float(num_strs[1])
        except:
            pass
        else:
            xdata.append(x)
            ydata.append(y)
            
    pyplot.plot(xdata, ydata)
    pyplot.autoscale()
    pyplot.xlabel('time (fortnight)')
    pyplot.ylabel('height (furlong)')
    pyplot.show()


while Running == True: 
    
    #if ser_port.is_open and ser_port.in_waiting:
        if state == 0:
            
            aline = ser_port.read().decode()
            print (aline)
        
    #else:
        ##prompt = 'Press C to input coordinates. Press F to fire a dart.'
        ##print(prompt)
        #command = input()
        
        ##if command == 'c':
        #command = command + '\r\n'
        #command = command.encode()
        #ser_port.write(command)
                
                #coordinate_instr = 'Enter the row and column of the destination square with the origin bottom left (row,column): '
                #print(coordinate_instr)
                
                #coordinates = input()
                #coordinates = coordinates + '\r\n'
                #coordinates = coordinates.encode()
                #ser_port.write(coordinates)
                #print ('Coordinates have been entered.')
                #state = 4
            
            #if command == 'f':
                #command = command + '\r\n'
                #command = command.encode()
                #ser_port.write(command)
                #print ('Shot fired!')
                
                
        #if(state == 1): 
            
            #setpoint = setpoint + '\r\n'
            #setpoint = setpoint.encode()
            #ser_port.write(setpoint)
            #aline = 'Ready'
            #mylist = []
            #while(aline != '') : 
                #aline = ser_port.readline().decode('UTF_8')
                #mylist.append(aline)
                #print(aline)
            #setpoint = setpoint.decode()
            #state = 2
            #count = 0
        #if(state == 2):
            #PlotData(mylist)
            #ser_port.flush()
            #state = 3
        #if(state == 3):
            #setpoint = input()
            #state = 1
            
        #if state == 4:
            #time.sleep(3)
            #state = 0
            
        #if state == 5:
            #readSerPort()
            #time.sleep(.2)
        

#for i in range(len(mylist)): 
 #   print(mylist[i])
    
        
    
    
