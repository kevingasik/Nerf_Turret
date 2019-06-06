def task_interface_fun ():
    ''' Function that implements the Interface Task. This task will continually
    check the serial port for instructions sent from a PC about where to aim
    and when to fire.'''

    state = 0
    line = ''
    #main_menu()
    char_in = None
    
    while True:
         
        if vcp.isconnected() == True and vcp.any():
            char_in = vcp.read()
            char_in = char_in.decode()
            line = line + char_in
            
            # The Interface is in the main menu of options.
            if state == 0:
                    
                if char_in == 'g':
                    
                    vcp.write('\r\nPlease enter coordinates:\r\n'.encode())
                    line = ''
                    state = 1
        
                elif char_in == 'f':
                    fire.put(1)

                elif char_in == '\x03':
                    run_RTOS.put (0)
                    
                elif char_in == 's':
                    vcp.write('Span Motor selected for direct control.\r\n'.encode())
                    vcp.write('Press F to operate forward (clockwise).\r\n'.encode())
                    vcp.write('Press R to operate in reverse (counter-clockwise).\r\n'.encode())
                    vcp.write('Press S to stop.\r\n'.encode())
                    vcp.write('Press E to return to the main menu.\r\n'.encode())
                    state = 4
                        
                elif char_in == 't':
                    vcp.write('Tilt Motor selected for direct control.\r\n'.encode())
                    vcp.write('Press F to operate forward (clockwise).\r\n'.encode())
                    vcp.write('Press R to operate in reverse (counter-clockwise).\r\n'.encode())
                    vcp.write('Press S to stop.\r\n'.encode())
                    vcp.write('Press E to return to the main menu.\r\n'.encode())
                    state = 5
                    
                elif char_in == 'i':
                    
                    x_imu = imu_x.get()
                    y_imu = imu_y.get()
                    z_imu = imu_z.get()
                    
                    
                    vcp.write('\r\nimu_x = '.encode())
                    vcp.send(str(x_imu).encode())
                    
                    vcp.write('\r\nimu_y = '.encode())
                    vcp.write(str(y_imu).encode())
                    
                    vcp.write('\r\nimu_z = '.encode())
                    vcp.write(str(z_imu).encode())
                
                    
                    vcp.write('\r\n'.encode())
                    
                elif char_in == 'c':
                    vcp.write('Please enter the following information in this format: shot1_x_offset, shot1_z_offset, shot2_z_diff\r\n'.encode())
                    line = ''
                    run_calibration.put(1)
                    state = 3
                
                #elif char_in == 'h':
                    #main_menu()
                
            # The interface is gathering which grid square to fire at.    
            elif state == 1:
                
                vcp.write(char_in)
                
                if char_in == '\r': 
                    vcp.write('\n'.encode())

                    coordinates = line.split(',')

                    y = coordinates[0]
                    x = coordinates[1]                   
                    
                    x = int(x)
                    y = int(y)
                    dest_row.put (y)
                    dest_col.put (x)
                    
                    calc_setpoint.put (1)
                    state = 0
                    
            
            # The interface is gathering calibration data from the user
            # which it will then send to the calculator.
            elif state == 3:
                
                vcp.write(char_in)
                
                if char_in == '\r': 
                    vcp.write('\n'.encode())

                    coordinates = line.split(',')

                    x = coordinates[0]
                    y = coordinates[1]                   
                    vcp.write(x.encode())
                    vcp.write(y.encode())
                    calc_setpoint.put (1)
                    x = float(x)
                    y = float(y)
                    dest_row.put (y)
                    dest_col.put (x)
                    
                    vcp.write('Calibrating...\r\n'.encode())
                    main_menu()
                    calibrate_calc.put(1)
                    state = 0
            
            # The interface is directly operating the span motor.
            elif state == 4:
                
                vcp.write(char_in)
                
                if char_in == 'f':
                    vcp.write('Span motor turning CW.'.encode())
                    span_mot_op.put(1)
                elif char_in == 'r':
                    #vcp.write('Span motor turning CCW.'.encode())
                    span_mot_op.put(2)
                elif char_in == 's':
                    #vcp.write('Span motor stopped.'.encode())
                    span_mot_op.put(0)
                elif char_in == 'e':
                    main_menu()
                    state = 0
            
            # The interface is directly operating the tilt motor.
            elif state == 5:
                
                vcp.write(char_in)
                
                if char_in == 'f':
                    #vcp.write('Tilt motor turning CW.'.encode())
                    tilt_mot_op.put(1)
                elif char_in == 'r':
                    #vcp.write('Tilt motor turning CCW.'.encode())
                    tilt_mot_op.put(2)
                elif char_in == 's':
                    #vcp.write('Tilt motor stopped.'.encode())
                    tilt_mot_op.put(0)
                elif char_in == 'e':
                    main_menu() 
                    state = 0
                    
            if char_in == '\r':
                    line = ''
            yield (state)
