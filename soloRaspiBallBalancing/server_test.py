import os 
import serial,time

ser = serial.Serial("COM7",115200,timeout=1)
print ("check which port was really used >",ser.name)

ser.write(b'AT\r')
time.sleep(0.5)
X = ser.read(20)
print(X)
#
ser.write(b'AT+Cnmp=109\r')
time.sleep(0.5)
X = ser.read(20)
print(X)
ser.write(b'AT+Cnmp?\r')
time.sleep(0.5)
X = ser.read(30)
print(X)
ser.write(b'AT+COPS?\r')
time.sleep(0.5)
X = ser.read(100)
print(X)

ser.close()