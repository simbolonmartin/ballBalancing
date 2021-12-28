import socket
import time
from control_sys import four_axis_convert
import math
import globals
import csv
class Ethernet:
    """socket define and internet address setting,
    including socket initialize/bind/listen.
    """
    def __init__(self, ip_type=2):
        self.ethernet = "127.0.0.1" #ethernet set:local
        self.ip_type = ip_type
        self.socket_init = 0
    def IP_set(self):
        if self.ip_type == 0:
            self.ethernet = '192.168.1.10'  # 有線(wire lan)
            print("set WLAN...")
        elif self.ip_type == 2:
            self.ethernet = '140.118.137.48' # wifi
            # self.ethernet = '192.168.0.108' #
            print("set wifi...")
        elif self.ip_type == 1:
                    
            self.ethernet = '221.120.83.21'  # 4G
            print(self.ethernet)
            print("set 4G...")
        elif self.ip_type == 3:
            self.ethernet = '192.168.225.20'
            # self.ethernet = '221.120.83.116'  # 5G
            print("set 5G...")
        else:
            print("error.")

    def socket_set(self, port=1111):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # socket set,for internet port preset
        print(self.ethernet)
        s.bind((self.ethernet, port))  # 把套接字綁定到ip地址
        s.listen(4)  # wait for 4 Raspberry Pi(motors) connect...
        print('Socket now listening')
        return s

def RPI_communication(conn):
    """this function is for data receiving/sending,
    any data packet declare will be here. """
    theta_ID = [0., 0., 0., 0.]
    small_angle = [0., 0., 0., 0.]
    final_send_angle = [0., 0., 0., 0.]
    sign = [0., 0., 0., 0.]
    while True:
        theta_ID[0], theta_ID[1], theta_ID[2], theta_ID[3] = four_axis_convert()
        for i in range(0, 4):
            if theta_ID[i] < 0:
                sign[i] = 0
            else:
                sign[i] = 1
            small_angle[i], final_send_angle[i] = math.modf(theta_ID[i])
            if abs(final_send_angle[i]) > 90:  # protection for motor
                final_send_angle[i] = 90
            small_angle[i] = int(small_angle[i] * 100)

        test = [sign[0], abs(int(final_send_angle[0])), abs(small_angle[0]),
                sign[1], abs(int(final_send_angle[1])), abs(small_angle[1]),
                sign[2], abs(int(final_send_angle[2])), abs(small_angle[2]),
                sign[3], abs(int(final_send_angle[3])), abs(small_angle[3]), 0x01]

        my_bytes = bytearray(test)
        conn.send(my_bytes)
        # time.sleep(1)   #test delay
        
        
        
        X = conn.recv(8)  # checking flag:make sure the data sends successfully
        while (X[0] != 0x01):
            print("wait.")
        # if(X[0]==0x01):
        #    print("get.")
        # time.sleep(0.001)
        time.sleep(0.001)

class extra_fn:
    """
    future developing function:
    save_CSV-> save the ball's position data in CSV file
    plot_line_chart-> plot out the distance of ball's position, with matplolib
    diagnosis -> analysis why the system can't work,checking these sub-system:1.internet,2.camera.
    """
    def __init__(self):
        self.csvfile = 0
    def save_CSV(self):
        writer = csv.writer(self.csvfile)
        writer.writerow(['TIME', 'X error', 'Y error'])
        globals.start_TIME = time.time() #initial starting time
    def plot_line_chart(self):
        print("Plot Line Chart")
    def diagnosis(self):
        print("Diagnosis")
