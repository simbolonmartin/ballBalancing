import cv2
import numpy as np
import globals
import math
import time
import csv
import sympy
import math
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread


class control_servo:
    """put control system algorithm over here.
       definition:PID&decision tree/SMC"""
 


    def Sliding_Mode_Control(self, error_x, error_y):
        #the error should be in meter, so we need to convert error_x and error_y to meter
        # print(error_x, error_y)
        error_x = error_x / 2000    #conversion to meter
        error_y = error_y / 2000       
        # print(error_x, error_y)
        # error_x = 0    #if you want to test in steady mode 
        # error_y = 0  
        # print(error_x, error_y)
        # print("1")
        # M = 0.244
        # r = 0.02
        # g = 9.8 
        # # I = 4.208*math.pow(10,-7)
        # I = 0.4 * M * r * r
        
        # smc parameter old
        # lambda_smc_x = 1.5 #origin 2.5
        # lambda_smc_y = 1
        # Kx = 11 
        # Ky = 10
        # psi = 0.99
       

        lambda_smc_x = 2 #origin 2.5
        lambda_smc_y = 2
        Kx = 4  
        Ky = 4
        psi = 0.99

        # SMC algorithm

        # X-axis and Y-axis s plane sliding mode function
        errorx_1dev = (error_x - globals.error_x_old) / (time.time() - globals.passtime)
        s_x = errorx_1dev + lambda_smc_x * error_x
        errory_1dev = (error_y - globals.error_y_old) / (time.time() - globals.passtime)
        s_y = errory_1dev + lambda_smc_y * error_y      
        
        # SMC dynamic equation
        globals.degree_x = 1/0.1633*(-lambda_smc_x*errorx_1dev + (Kx * s_x) / (abs(s_x)+ psi) )
        globals.degree_y = 1/0.1633*(-lambda_smc_y*errory_1dev + (Ky * s_y) / (abs(s_y)+ psi) )


        # globals.degree_x = 1/0.39*(-lambda_smc_x*errorx_1dev + (Kx * s_x) / (abs(s_x)+ psi) )
        # globals.degree_y = 1/0.39*(-lambda_smc_y*errory_1dev + (Ky * s_y) / (abs(s_y)+ psi) )


        globals.sliding_surface_x = s_x
        globals.sliding_surface_y = s_y
        globals.error_x_dot = errorx_1dev
        globals.error_y_dot = errory_1dev

        # if (abs(globals.degree_x) > 3 ):
            # print(globals.degree_x, globals.degree_y)
        # elif (abs(globals.degree_y) > 3 ):
        #     print(globals.degree_x, globals.degree_y)
        # # print("globals.degree_x, globals.degree_y")
        # print("Before:")
        # print(globals.degree_x, globals.degree_y)

        globals.error_x_old = error_x
        globals.error_y_old = error_y 
        globals.passtime = time.time()       


    def Modified_Sliding_Mode_Control(self, error_x, error_y):
        #the error should be in meter, so we need to convert error_x and error_y to meter
        # print(error_x, error_y)

        # error_x = error_y           #convert
        # error_x = error_x / 2000    #conversion to 
        


        #for 1 dof diagonally
        error_x = error_x / 2000



        # print(error_x)
        # error_y = error_y / 2000        
        # print(error_x, error_y)
        # print("Error and Old")
        # print(error_x)
        # print(globals.error_x_old)

        # error_x = 0
        # print(error_x, error_y)
           
        #2.1179    0.0081    4.7081   19.8914   50.3857
        # 4.3074   12.6504	5.8695   17.6218  207.3382

        # 13.73711      1.517444      0.729935      254.3225      52.41114
        # sigma1_x = 4.3074 #P
        # sigma2_x = 12.6504    #I
        # sigma3_x = 5.8695   #D
        # varphi_x = 17.6218
        # Kx = 207.3382
        
        #13.73711      1.517444      0.729935      254.3225      52.41114 #1 dof diagonally reducedBES-best
        #3.249445     0.1067807      5.121587      14.41063       267.021
        #2.2369    0.0620    3.0671    3.2276   60.2545
        sigma1_x = 2.2369  #P
        sigma2_x = 0.0620   #I
        sigma3_x = 3.0671    #D
        varphi_x = 3.2276
        Kx = 60.2545
        

        if globals.deltaTimeCounterInit < 10:
            deltaTime = 0
            errorx_1dev = 0
            deltaSum = 0
            globals.deltaTimeCounterInit += 1
        else:
            deltaTime = (time.time() - globals.passtime)
            errorx_1dev = (error_x - globals.error_x_old) / deltaTime
            deltaSum = (globals.error_x_old + error_x)/2 * deltaTime

        # print(globals.deltaTimeCounterInit)
        # print(deltaTime)
        # X-axis and Y-axis s plane sliding mode function
        # errorx_1dev = (error_x - globals.error_x_old) / deltaTime
        # errory_1dev = (error_y - globals.error_y_old) / (time.time() - globals.passtime)
        
        globals.error_x_dot = errorx_1dev
        # globals.error_y_dot = errory_1dev

        # print(deltaTime)
        # deltaSum = (globals.error_x_old + error_x)/2 * deltaTime
        # if globals.counterTestSumError == 0:
        #     print("init")
        #     print(globals.sum_error_x)
        #     print(globals.error_x_old)
        #     print(deltaSum)
            
        
        globals.sum_error_x = globals.sum_error_x + (deltaSum) 
        # globals.sum_error_y = globals.sum_error_y + ((globals.error_y_old + error_y)/2 * (time.time()-globals.passtime))
        # if globals.counterTestSumError == 0:
        #     print(globals.sum_error_x)  
        # globals.counterTestSumError = 1


        # part1 = error_x * sigma1_x
        # part2 = globals.sum_error_x * sigma2_x
        # part3 = errorx_1dev * sigma3_x
        # print(part1)
        # print(part2)
        # print(part3)
        # print("----")
        p = error_x * sigma1_x
        i = globals.sum_error_x * sigma2_x
        d = errorx_1dev * sigma3_x
        s_x = error_x * sigma1_x + globals.sum_error_x * sigma2_x + errorx_1dev * sigma3_x


        # s_y = error_y * sigma1_y + globals.sum_error_y * sigma2_y + errory_1dev * sigma3_y 
        # globals.sliding_surface_x = s_x
        # globals.sliding_surface_y = s_y  
        # print(s_x)

        saturationInput =s_x/varphi_x
        # print(saturationInput)
        # SMC dynamic equation
        sat_x = np.clip(saturationInput, a_min = - 1, a_max = 1) 
        # sat_y = np.clip(s_y/varphi_y , a_min = - 1, a_max = 1) 
        # clippedSat_x = sat_x

        sat_x = Kx * sat_x
        globals.saturation_x = sat_x
        # globals.saturation_y = sat_y * Ky
        # print("---")
        # print(sat_x)
        # sat_x = 0
        group1 = sigma1_x * errorx_1dev + sigma2_x * error_x + sat_x
        # print(sigma1_x * errorx_1dev + sigma2_x * error_x )
        # print(group1)
        degree_x_noLimit = 1/(sigma3_x*0.4496) * group1 *180 / math.pi  #it is actually radian, so we need to convert it again to degree 


        # globals.degree_y = 1/(sigma3_y* 0.39) * (sigma1_y * errory_1dev + sigma2_y * error_y + Ky * sat_y)      
        # globals.degree_x =  group1 / 2
        # globals.degree_y = 0
        # print("--")
        # print(degree_x_noLimit)


        number = 30

        limitPos = number
        limitNeg = -number
            # time.sleep(1)   #test delay
        if degree_x_noLimit > limitPos:
            degree_x_noLimit = limitPos
        if degree_x_noLimit < limitNeg:
            degree_x_noLimit = limitNeg


        
        globals.degree_x = degree_x_noLimit    #to motor, not plate
        
        print(degree_x_noLimit)
        # print("%f %f %f %f %f %f"%(degree_x_noLimit, error_x, saturationInput, p, i, d))

        #in controller
        # print("1")
        # print(globals.degree_x)
        # print(" %f  %f" % (sat_x, sigma1_x * errorx_1dev + sigma2_x * error_x))
        # print("%f %f %f %f" % (globals.passtime, error_x, globals.degree_x, s_x))
        globals.control_signal_x = globals.degree_x
        globals.control_signal_y = globals.degree_y

        globals.error_x_old = error_x
        globals.error_y_old = error_y 
        globals.passtime = time.time()


class vision_servo(control_servo):
    """"""
    def __init__(self):
        self.degree_X = 0
        self.degree_Y = 0

        #old
        # self.origin_x = 325
        # self.origin_y = 230

        #new
        # self.origin_x = 236
        # self.origin_y = 236
        dif_x = 0
        dif_y = 0

        # self.origin_x = 235 + dif_x
        # # self.origin_y = 207 + dif_y #original at center
        # # self.origin_y = 445
        # self.origin_y = 425  
        self.origin_x = 258
        self.origin_y = 233


        self.last_measurement = np.array((2, 1), np.float32)
        self.current_measurement = np.array((2, 1), np.float32)
        self.last_predicition = np.zeros((2, 1), np.float32)
        self.current_prediction = np.zeros((2, 1), np.float32)
        self.csvfile = 0

    def ball_predict(self, event, x, y, kalman):
        # print("Position:")
        # print(x)
        # print(y)



        # initialize kalman parameters
        self.last_measurement = self.current_measurement
        self.last_prediction = self.current_prediction
        self.current_measurement = np.array([[np.float32(x)], [np.float32(y)]])
        # revise the prediction of kalman filter
        kalman.correct(self.current_measurement)
        # 呼叫kalman這個類的predict方法得到狀態的預測值矩陣，用來估算目標位置
        self.current_prediction = kalman.predict()
        # last time step's data
        lmx, lmy = self.last_measurement[0], self.last_measurement[1]
        # current time step's data
        cmx, cmy = self.current_measurement[0], self.current_measurement[1]
        # last time step's result
        lpx, lpy = self.last_prediction[0], self.last_prediction[1]
        # predict next time step's result
        cpx, cpy = self.current_prediction[0], self.current_prediction[1]
        # drawing on the video frame
        cv2.circle(event, (int(cpx), int(cpy)), 3, (0, 255, 0), 6)
        cv2.rectangle(event, (cpx - 35, cpy - 35), (cpx + 35, cpy + 35), (0, 255, 0), 2)
        # print(cpx, cpy) # print position

    def camera(self):
        # -----parameters initial------#
        # PID Dat
        error_x_old = 0.  # last error data
        error_y_old = 0.
        #
        # -----------KALMAN SET--------#
        kalman = cv2.KalmanFilter(4, 2)
        # setting measurement matrix
        kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        # setting transition matrix
        kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        # setting noise covariances matrix
        kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.03
        # -----------------------------#
        cam = cv2.VideoCapture(0)
        
        # print("old X:" + str(globals.degreeX_old) + ", old Y:" + str(globals.degreeY_old))
        while (cam.isOpened()):  # wait until the camera is 
            
            # startTime = time.time()
            ret, img = cam.read()
            # print("Camera Time : %s seconds" %(time.time() -  startTime))
            img = img[:, 50:560]
            error_x, error_y = self.image_process(img, kalman)  # image processing, get the ball's position

            # print(error_x,error_y)
            # self.tree_PID(error_x,error_y)
            # self.new_PID(error_x,error_y)
            # self.Sliding_Mode_Control(error_x, error_y)
            self.Modified_Sliding_Mode_Control(error_x, error_y)

            # number = 1

            # limitPos = number
            # limitNeg = -number
            # # time.sleep(1)   #test delay
            # if globals.degree_x > limitPos:
            #     globals.degree_x = limitPos
            # if globals.degree_x < limitNeg:
            #     globals.degree_x = limitNeg
            # if globals.degree_y > limitPos:
            #     globals.degree_y = limitPos
            # if globals.degree_y < limitNeg:
            #     globals.degree_y = limitNeg

            # print("After:")
            # print(globals.degree_x)
            # print(globals.degree_x, globals.degree_y)
            

            # globals.degree_x = 0
            # globals.degree_y = 0
            # print("errorX:" + str(error_x) + "errorY:" + str(error_y))
            # print("X_flag:" + str(globals.X_flag) + ",Y_flag:" + str(globals.Y_flag))
            # print("-------line-------")
            # if cv2.waitKey(10) & 0xFF == ord('q'):
            #     cam.release()
            #     cv2.destroyAllWindows()
            #     
            
            # controlProcessingTimefromCamera = (time.time() -  startTime)
            # print(controlProcessingTimefromCamera)

    def image_process(self, img, kalman):
        """get the ball's position,
        and some simple drawing on the frame.
        parameter->
            cX,cY: the ball's center position
            error_X,error_Y:the pixel difference between center and ball position
            """
        HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([74,0,246])     #pingpong white
        upper = np.array([179,242,255])


        # lower = np.array([0,110,153])     #pingpong
        # upper = np.array([99,240,255])

        # lower = np.array([0,0,181])     #metalball last used
        # upper = np.array([179,176,255])

        #metalball update
        #0 95 0 240 229 255
        # lower = np.array([0,0,229])     #metalball
        # upper = np.array([95,240,255])

        
        HSV_clip = cv2.inRange(HSV,lower,upper)

        # HSV_clip = np.clip(HSV[..., 2], 88, 255)
        # print(HSV)
        HSV_Blur = cv2.GaussianBlur(HSV_clip, (5, 5), cv2.BORDER_DEFAULT)
        kernel = np.ones((3, 3), np.uint8)
        erosion = cv2.erode(HSV_Blur, kernel, iterations=2)
        ret, threshold2 = cv2.threshold(erosion, 50, 255, cv2.THRESH_BINARY_INV)
        #######################################################################
        _, contours, hierarchy = cv2.findContours(HSV_Blur, cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
    
        i = 0
        areas = []
        for c in contours:
            A = cv2.contourArea(c)
            if A < 5500.:
                areas.append(A)

        if len(areas) != 0:
            max_index = np.argmax(areas)
            contours = contours[max_index]
        else:
            contours = globals.old_contours

        
        M = cv2.moments(contours)
        if M["m00"] == 0:
            M["m00"] = 1536375  # 讓球被移除時不要讓被除數等於0 to make sure when the ball is remove,there will still be an select point
            cX, cY = 0, 0
        else:
            cX = int(M["m10"] / M["m00"])  # calculate x,y coordinate of center
            cY = int(M["m01"] / M["m00"])  # 算質心
        # 绘制轮廓

        globals.old_contours = contours
        # cv2.drawContours(img, contours, -1, (0, 0, 255), 3, lineType=cv2.LINE_AA)
        cv2.circle(img, (cX, cY), 7, (0, 0, 255), -1)
        cv2.rectangle(img, (int(cX - 35), int(cY - 35)), (int(cX + 35), int(cY + 35)), (255, 0, 0), 2)

        # print('目前求的圓心: ',cX, cY)
        cv2.circle(img, (int(cX), int(cY)), 3, (255, 0, 0), 6)
        cv2.rectangle(img, (int(cX - 35), int(cY - 35)), (int(cX + 35), int(cY + 35)), (255, 0, 0), 2)
        self.ball_predict(img, cX, cY, kalman)

        # print(cY)
        # half screen old
        # cv2.line(img, (0, 240), (640, 240), (0, 255, 0), 2)     
        # cv2.line(img, (240, 0), (240, 480), (0, 255, 0), 2)
        # cv2.line(img, (0, 235), (640, 235), (0, 255, 0), 2)   #horizontal line
        # cv2.line(img, (240, 0), (240, 480), (0, 255, 0), 2)     #vertical line top bottom
        cv2.line(img, (0, 233), (640, 233), (0, 255, 0), 2)   #horizontal line
        cv2.line(img, (258, 0), (258, 480), (0, 255, 0), 2)   #vertical line


        # centerX= 207
        # centerY= 235 
        centerX= 258
        centerY= 233 

        # centerY = #change y to 0.1, so the target become 0.1 from 0
        sideLength = 50
        cv2.rectangle(img, (centerX-sideLength, centerY-sideLength), (centerX+sideLength, centerY+sideLength), (255, 0, 0), 2)

        # cv2.circle(, (207,235), 40, (255, 0, 0), )

        self.change_pixmap_signal.emit(img)
        # self.change_pixmap_signal.emit(img)
        error_x = self.origin_x - cX
        error_y = self.origin_y - cY
        # print("%d, %d"%(cX, cY))
        # print("%d, %d"%(error_x, error_y))

        #for one degree of freedom
        error_x = math.sqrt(error_x*error_x+error_y*error_y)
        if error_y < 0:
            error_x = -error_x

        # print(error_x)
        

        # print("position:(" + str(cX) + "," + str(cY) + ")")

        # # write data in csv file
        # if globals.record_times>0:
        #     if globals.save_file_signal and globals.camera_file_open and globals.finish_initialize:
        #         globals.camera_file_open = False
        #         globals.csvfile = open(globals.file_name, 'a', newline='')
        #         # globals.start_TIME = time.time()
        #     elif globals.save_file_signal and not globals.end_signal and globals.game_flag and globals.finish_initialize:
        #         if time.time()-self.last_time_period>=0.1:
        #             writer = csv.writer(globals.csvfile)
        #             writer.writerow([time.time() - globals.start_TIME, error_x, error_y, cX, cY])
                
        #             self.last_time_period = time.time()
        #     elif globals.save_file_signal and not globals.end_signal and not(globals.game_flag) and globals.CSV_recorder_timer and globals.finish_initialize and globals.record_times>1:
        #         writer = csv.writer(globals.csvfile)
        #         writer.writerow(["-----","balance duration",time.time()-globals.start_TIME,"-----"])
        #         writer.writerow(["-----","-----","END","-----","-----"])
        #         writer.writerow([""])
        #         globals.CSV_recorder_timer = False
        #     elif globals.save_file_signal and globals.end_signal:
        #         print(globals.csvfile)
        #         writer = csv.writer(globals.csvfile)
        #         balance_mean_time = sum(globals.balance_recorder)/len(globals.balance_recorder)
        #         writer.writerow("-----","-----","End","-----","-----")
        #         writer.writerow(["-----","-----","average balance time",balance_mean_time])
        #         globals.csvfile.close()

        #-------------------------------------------------------------#
        # print(globals.degree_x, error_x)
            
        return error_x, error_y


def four_axis_convert():
    """
    unit:degree / cm
    read in platform angle, send out motor degree:
    degree_x, degree_y = map(float,input("expected platform degree (X,Y):").split())
    print("degree_x:{},degree_y:{}".format(degree_x,degree_y))
    4 vertices:(-12,-12),(-12,12),(12,-12),(12,12)
    platform Length and width:24, set 12 for coordinate
    """
    # X_long = 24 / 2
    # Y_long = 24 / 2
    # # change to degree(preset is rad)
    # """cos_thetaY = math.cos(degree_y*math.pi/180)
    # sin_thetaY = math.sin(degree_y*math.pi/180)
    # cos_thetaX = math.cos(degree_x*math.pi/180)
    # sin_thetaX = math.sin(degree_x*math.pi/180)"""


    # #with conversion
    # cos_thetaY = math.cos(globals.degree_x * math.pi / 180)
    # sin_thetaY = math.sin(globals.degree_x * math.pi / 180)
    # cos_thetaX = math.cos(globals.degree_y * math.pi / 180)
    # sin_thetaX = math.sin(globals.degree_y * math.pi / 180)

    # #without conversion
    # # cos_thetaY = math.cos(globals.degree_x)
    # # sin_thetaY = math.sin(globals.degree_x)
    # # cos_thetaX = math.cos(globals.degree_y)
    # # sin_thetaX = math.sin(globals.degree_y)

    # # original string coordinate set-> can neglect at first, due to small angle changing.
    # # get Z -axis longtitude
    # X_forz = np.array([[12., -12.], [-12., 12.]])
    # Y_forz = np.array([[12., 12.], [-12., -12.]])
    # Z = -sin_thetaY * X_forz + sin_thetaX * cos_thetaY * Y_forz  # get present Z-coordinate
    # Z_2 = np.array([[0., 0.], [0., 0.]])  # original Z data
    
    
    # # print("Z: data")
    # # print(Z)
    # # print(Z_2)
    # # reckon the angle send to motor:
    # """remenber, each motor has different preset of exchanging function
    # ID1,ID4->0 degree, clockwise//ID2->0,degree, counterwise//ID3->180 degree, counterwise
    # motor diameter:35 mm"""
    
    
    
    # theta_ID1 = (Z[0][0] - Z_2[0][0]) * (180. / 3.5) / 2  # count it again.Something wrong, but not serious problem.
    # theta_ID2 = (Z[0][1] - Z_2[0][1]) * (180. / 3.5) / 2
    # theta_ID3 = (Z[1][0] - Z_2[1][0]) * (180. / 3.5) / 2
    # theta_ID4 = (Z[1][1] - Z_2[1][1]) * (180. / 3.5) / 2
    # print("4 motor angle send:1->{},2->{},3->{},4->{}".format(theta_ID1, theta_ID2,theta_ID3,theta_ID4))

    
    # if globals.counterDegree < 100:
    #     degree_x = 0
    # elif globals.counterDegree < 200:
    #     degree_x = 0.5
    # elif globals.counterDegree < 300:
    #     degree_x = 1.0
    # elif globals.counterDegree < 400:
    #     degree_x = 1.5
    # elif globals.counterDegree < 900:
    #     degree_x = 2.0
    # else:
    #     globals.counterDegree = 0
    #     degree_x = 0

    
            



    #with transformation from alpha to theta
    # # degree_x = -10
    # # globals.degree_x = 0.44
    # # print("In convert")
    # # print("2")
    # degree_x_send = globals.degree_x
    # # print(degree_x_send)
    # # degree_x_send = 0
    # # globals.degree_x = globals.degree_x * 180 / math.pi
    # degree_x_send = 2 * degree_x_send - 1  #regression
    # theta_ID1 = 12.0 / 3.7 * degree_x_send
    # # theta_ID1 = 0
    # theta_ID2 = theta_ID1

    # theta_ID3 = - theta_ID1
    # theta_ID4 = theta_ID3
    # print(globals.degree_x)
    theta_ID1 = globals.degree_x
    theta_ID3 = -theta_ID1
    theta_ID2 = 0
    theta_ID4 = 0

    #no transformation



    
     

    # origin motor set changing:
    theta_ID1 = -theta_ID1
    theta_ID3 = -theta_ID3
    # print("4 real motor angle send:1->{},2->{},3->{},4->{}".format(theta_ID1, theta_ID2,theta_ID3,theta_ID4))
    return theta_ID1, theta_ID2, theta_ID3, theta_ID4

def new_func():
    return 2.0
