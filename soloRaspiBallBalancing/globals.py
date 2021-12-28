import numpy as np
"""global variable setting block:
   Set your global variable,
   divided by their belonging function.
"""
def global_variable_init():
   # ------camera------ #
    global degreeX_old,degreeY_old,degree_x,old_contours, degree_y
    degree_x = degree_y = 0
    degreeX_old = 0
    degreeY_old = 0
    old_contours = [100,100]
    #------PID ------#
    #PID parameters
    global error_x_old,error_y_old,sum_error_x,sum_error_y
    error_x_old = 0
    error_y_old = 0
    sum_error_x = 0
    sum_error_y = 0
    #flag
    global X_flag_innertoolong,Y_flag_innertoolong,X_flag,Y_flag
    X_flag_innertoolong = 0
    Y_flag_innertoolong = 0
    X_flag = 0
    Y_flag = 0
    #------SMC------#
    global passtime
    passtime = 0
    global sliding_surface_x, sliding_surface_y, control_signal_x, control_signal_y, saturation_x, saturation_y, error_x_dot, error_y_dot
    sliding_surface_x = 0
    sliding_surface_y = 0
    control_signal_x = 0
    control_signal_y = 0
    saturation_x = 0
    saturation_y = 0
    error_x_dot = 0
    error_y_dot = 0
    #-----csv-----#
    global start_TIME,save_file_signal,camera_file_open,end_signal
    save_file_signal = False
    camera_file_open = True
    end_signal = False
    #-----ethernet setting-----#
    global ip_type
    global counterDegree
    global counterTestSumError
    counterTestSumError = 0
    counterDegree  = 0
    ip_type = 0
    print("initial done.")
    global deltaTimeCounterInit
    deltaTimeCounterInit = 0 
