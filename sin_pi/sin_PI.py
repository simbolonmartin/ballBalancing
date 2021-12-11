# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import serial
import socket
import time
from threading import Timer
import numpy as np
import threading
count = 0
#if use half-auto, EN_485 = LOW is Receiver, EN_485 = HIGH is Send
MODE = 0 #mode = 0 is full-guto, mode = 1 is half-auto
theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
timerCounter = 1
start_time = 0


if MODE == 1:
    EN_485 =  4
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(EN_485,GPIO.OUT)
    GPIO.output(EN_485,GPIO.HIGH)


ser = serial.Serial("/dev/ttyS0",115200,timeout=1) 

test01 = [0xff,0xff,0xfd,0x00,0x01,0x06,0x00,0x03,0x32,0x02,0x01,0x30,0xec] #pack
test_array01 = bytearray(test01)
test02 = [0xff,0xff,0xfd,0x00,0x02,0x06,0x00,0x03,0x32,0x02,0x01,0x00,0xef]
test_array02 = bytearray(test02)
test03 = [0xff,0xff,0xfd,0x00,0x03,0x06,0x00,0x03,0x32,0x02,0x01,0x13,0x6e]
test_array03 = bytearray(test03)
test04 = [0xff,0xff,0xfd,0x00,0x04,0x06,0x00,0x03,0x32,0x02,0x01,0x60,0xe9]
test_array04 = bytearray(test04)
while 1:
    if (count < 10):
        ser.write(test_array01)
        time.sleep(0.01)
        count+=1
    elif count < 20:
        ser.write(test_array02)
        time.sleep(0.01)
        count+=1
    elif count < 25:
        ser.write(test_array03)
        time.sleep(0.01)
        count+=1
    elif count < 30:
        ser.write(test_array04)
        time.sleep(0.01)
        count+=1
    else:
        break
#ser.flush()


MOTOR_CNT = 1
MOTOR_ID_1 = 1
MOTOR_ID_2 = 2
MOTOR_ID_3 = 3
MOTOR_ID_4 = 4 
MOTOR_ID_5 = 5
MOTOR_ID_6 = 6
MOTOR_ID_7 = 7
MOTOR_ID_8 = 8
MOTOR_ID_9 = 9
MOTOR_ID_10 = 10
MOTOR_ID_11 = 11
MOTOR_ID_12 = 12

motor_id =[]
motor_id.append(MOTOR_ID_1)
motor_id.append(MOTOR_ID_2)
motor_id.append(MOTOR_ID_3)
motor_id.append(MOTOR_ID_4)
motor_id.append(MOTOR_ID_5)
motor_id.append(MOTOR_ID_6)
motor_id.append(MOTOR_ID_7)
motor_id.append(MOTOR_ID_8)
motor_id.append(MOTOR_ID_9)
motor_id.append(MOTOR_ID_10)
motor_id.append(MOTOR_ID_11)
motor_id.append(MOTOR_ID_12)

posi = []
pos = []
vel = []
array =[]
MOTOR_CNT = 1
vel_data = 100


H54_040_pos = 2045#251417
H54_040_vel = 2840

def DXL_LOWORD(l):
    
    l = int(l) & 0xffff
    return l
def DXL_HIWORD(l):
    l = (int(l) >>16)& 0xffff
    return l
def DXL_LOBYTE(w):
    w = int(w) & 0xff
    return w
def DXL_HIBYTE(w):
    w = (int(w) >> 8) & 0xff
    return w

def update_crc(crc_accum,Pkg,Pkg_Size):

    crc_table = [0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202]

    for j in range(len(Pkg)):
        i = (crc_accum>>8) ^ Pkg[j] & 0xFF
#         print('Data : ',data[j],'  i : ',i,'Table : ',crc_table[i],'  (crc_accum << 8) : ',(crc_accum << 8))
        crc_accum = (crc_accum << 8) ^ crc_table[i]
        crc_accum = crc_accum & 0xFFFF
#         print('crc_accum : ',crc_accum)
    CRC_L = (crc_accum & 0x00FF)
    CRC_H = (crc_accum >> 8) & 0x00FF
    #print('CRC_H',CRC_H,'  CRC_L : ',CRC_L)
    CRC = [CRC_L,CRC_H]
    return CRC


def send_data(bridge):

    #angle = np.sin(x)*180
    #angle = bridge
    #print("angle:",bridge)
    #for z in range(len(angle)):
    
    pos.append (float((bridge/ 180.0)) * H54_040_pos)
    vel.append (float((vel_data / 100.0)) * H54_040_vel)

    array.append(0xFF) #0
    array.append(0xFF) #1
    array.append(0xFD) #2
    array.append(0x00) #3
    array.append(0xFE) #4
    array.append((1+4) * 1 + 7) #5
    array.append(0x00) #6
    array.append(0x83) #7
    array.append(0x54) #8
    array.append(0x02) #9
    array.append(0x04) #10
    array.append(0x00) #11
    #test1 = [0xff,0xff,0xfd,0x00,0x04,0x09,0x00,0x03,0x54,0x02,0xff,0x03,0x00,0x00,0x66,0x65]

    posi.append(90)
        #angle = input("Your angle: ")
   
    array.append(0x01)
    array.append(DXL_LOBYTE((pos[0])))
    array.append(DXL_HIBYTE((pos[0])))
    array.append(DXL_LOBYTE(DXL_HIWORD((pos[0]))))
    array.append(DXL_HIBYTE(DXL_HIWORD((pos[0]))))
    
        
    CRC = update_crc(0,array,len(array))

    array.append((CRC[0]))
    array.append((CRC[1]))   
    array_send = bytearray(array)
        #ser.write(array_send)
    ser.write(array_send)
    time.sleep(0.000001)    
    array_send.clear()
    array.clear()
    pos.clear()
        
def send_data3(bridge):
        
        pos.append (float((bridge/ 180.0)) * H54_040_pos)
        vel.append (float((vel_data / 100.0)) * H54_040_vel)

        array.append(0xFF) #0
        array.append(0xFF) #1
        array.append(0xFD) #2
        array.append(0x00) #3
        array.append(0xFE) #4
        array.append((1+4) * 1 + 7) #5
        array.append(0x00) #6
        array.append(0x83) #7
        array.append(0x54) #8
        array.append(0x02) #9
        array.append(0x04) #10
        array.append(0x00) #11
        #test1 = [0xff,0xff,0xfd,0x00,0x04,0x09,0x00,0x03,0x54,0x02,0xff,0x03,0x00,0x00,0x66,0x65]

        posi.append(90)
            #angle = input("Your angle: ")
       
        array.append(0x03)
        array.append(DXL_LOBYTE((pos[0])))
        array.append(DXL_HIBYTE((pos[0])))
        array.append(DXL_LOBYTE(DXL_HIWORD((pos[0]))))
        array.append(DXL_HIBYTE(DXL_HIWORD((pos[0]))))
        
            
        CRC = update_crc(0,array,len(array))

        array.append((CRC[0]))
        array.append((CRC[1]))
        
            
        array_send = bytearray(array)
            #ser.write(array_send)
        ser.write(array_send)
        time.sleep(0.000001)    
        array_send.clear()
        array.clear()
        pos.clear()

def send_data2(bridge):
    
    pos.append (float((bridge/ 180.0)) * H54_040_pos)
    vel.append (float((vel_data / 100.0)) * H54_040_vel)

    array.append(0xFF) #0
    array.append(0xFF) #1
    array.append(0xFD) #2
    array.append(0x00) #3
    array.append(0xFE) #4
    array.append((1+4) * 1 + 7) #5
    array.append(0x00) #6
    array.append(0x83) #7
    array.append(0x54) #8
    array.append(0x02) #9
    array.append(0x04) #10
    array.append(0x00) #11
    #test1 = [0xff,0xff,0xfd,0x00,0x04,0x09,0x00,0x03,0x54,0x02,0xff,0x03,0x00,0x00,0x66,0x65]

    posi.append(90)
        #angle = input("Your angle: ")
   
    array.append(0x02)
    array.append(DXL_LOBYTE((pos[0])))
    array.append(DXL_HIBYTE((pos[0])))
    array.append(DXL_LOBYTE(DXL_HIWORD((pos[0]))))
    array.append(DXL_HIBYTE(DXL_HIWORD((pos[0]))))
    
        
    CRC = update_crc(0,array,len(array))

    array.append((CRC[0]))
    array.append((CRC[1]))
    
        
    array_send = bytearray(array)
        #ser.write(array_send)
    ser.write(array_send)
    time.sleep(0.000001)        
    array_send.clear()
    array.clear()
    pos.clear()
# setting for H42 motor:
H42_20_pos = 151875
H42_20_vel = 2840
def send_data4(bridge ):
    pos.append (float((bridge/ 180.0)) * H42_20_pos)
    vel.append (float((vel_data / 100.0)) * H54_040_vel)

    array.append(0xFF) #0
    array.append(0xFF) #1
    array.append(0xFD) #2
    array.append(0x00) #3
    array.append(0xFE) #4
    array.append((1+4) * 1 + 7) #5
    array.append(0x00) #6
    array.append(0x83) #7
    array.append(0x54) #8
    array.append(0x02) #9
    array.append(0x04) #10
    array.append(0x00) #11
    #test1 = [0xff,0xff,0xfd,0x00,0x04,0x09,0x00,0x03,0x54,0x02,0xff,0x03,0x00,0x00,0x66,0x65]

    posi.append(90)
        #angle = input("Your angle: ")
   
    array.append(0x04)
    array.append(DXL_LOBYTE((pos[0])))
    array.append(DXL_HIBYTE((pos[0])))
    array.append(DXL_LOBYTE(DXL_HIWORD((pos[0]))))
    array.append(DXL_HIBYTE(DXL_HIWORD((pos[0]))))
    
        
    CRC = update_crc(0,array,len(array))

    array.append((CRC[0]))
    array.append((CRC[1]))
    
        
    array_send = bytearray(array)
        #ser.write(array_send)
    ser.write(array_send)
    time.sleep(0.000001)    
    array_send.clear()
    array.clear()
    pos.clear()
    
def send_total():
    send_data(theta1)
    send_data2(theta2)
    send_data3(theta3)
    send_data4(theta4)
    Timer(0.001,send_total).start()
    
    global timerCounter, start_time
    if timerCounter == 1:
        start_time = time.time()
        timerCounter = 2
    elif timerCounter == 2:
        print("%s seconds " %(time.time() - start_time))
        timerCounter = 1
    
    
    
    
def doconnect():
    host_wlan = "192.168.1.10"  
    host_wifi = "140.118.137.48"
    host_5G = "221.120.83.116"
    host_4G = "192.168.225.78"
    addr_wlan = (host_wlan,1111)
    addr_wifi = (host_wifi,1111)
    addr_4G = (host_4G,1111)
    addr_5G = (host_5G,1111)

    while 1:
        try:
            print("Now trying connent to wlan...")
            client = socket.socket()
            client.settimeout(3.0)
            client.connect(addr_wlan)
            break
        except:
            print("try wifi...")
            client.close()
            time.sleep(0.3)      
        try:
            print("Now trying connent to wifi...")
            client = socket.socket()
            client.settimeout(3.0)
            client.connect(addr_wifi)
            break
        except:
            print("try 4G...")
            client.close()
            time.sleep(0.5)
        try:
            print("Now trying connent to 4G...")
            client = socket.socket()
            client.settimeout(3.0)
            client.connect(addr_4G)
            break
        except:
            print("try 5G...")
            client.close()
            time.sleep(0.5)
        try:
            print("Now trying connent to 5G...")
            client = socket.socket()
            client.settimeout(3.0)
            client.connect(addr_5G)
            break
        except:
            print("retry...wait for server to open...")
            client.close()
            time.sleep(5)        

    return client
def get_data(client):
    global theta1,theta2,theta3,theta4
    get_data = [0.,0.,0.,0.]
    try:
        receivedata = client.recv(256)
    except socket.error:
        print("\n\nconnection failed.")
        client = doconnect()
        receivedata = client.recv(256)
    mybyte = bytearray([0x01,0x02])
    while(receivedata[12]!=0x01):
        #receivedata = client.recv(256)
        print("no no")
    if receivedata[12] == 0x01:
        print("get")
        client.send(mybyte)
    #ord(receivedata[3])  
    #time.sleep(0.001)
    
    
        
    print(receivedata)
    #print("big angle:"+str(receivedata[3]))
    #print(ord(receivedata[3]))
    #print(type(receivedata[3]))
    get_data[0] = float(receivedata[1])+(float(receivedata[2])/100.)
    get_data[1] = float(receivedata[4])+(float(receivedata[5])/100.)
    get_data[2] = float(receivedata[7])+(float(receivedata[8])/100.)
    get_data[3] = float(receivedata[10])+(float(receivedata[11])/100.)
    if receivedata[0]==0:
        get_data[0] = -get_data[0]
    if receivedata[3]==0:
        get_data[1] = -get_data[1]
    if receivedata[6] ==0:
        get_data[2] = -get_data[2]        
    if receivedata[9] ==0:
        get_data[3] = -get_data[3]         
    #time.sleep(0.01)
    theta1 = get_data[0]
    theta2 = get_data[1]
    theta3 = get_data[2]
    theta4 = get_data[3]
    receivedata = 0
    return client
        
if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)   
    i=0
    GPIO.setup(11, GPIO.OUT)
    while i<3:
       i+=1
       GPIO.output(11,True)  
       time.sleep(0.01)    
       GPIO.output(11,False)
       time.sleep(0.01)
       print("-----")

    client = doconnect()
    print("connect success!")

    i = 0
    time.sleep(0.01)
    t = Timer(0.001,send_total)
    t.start()
    while(1):
        client = get_data(client)
        print(i)
        i+=1
