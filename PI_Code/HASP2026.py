from time import sleep
import numpy
from ModeControl import ModeControl
from ModeControl import SystemMode
import threading
import serial
import random
import struct
from getch import getch
import sys
import sqlite3
import queue
from adafruit_extended_bus import ExtendedI2C as I2C

def input_worker_thread():
    while True:
        #print("Input\n")
        global stop_input_thread
        global command_in
        global sensorQueue

        # Enqueue sensor data
        for i in range(1):
            j = i % 3

            while not i2cbus.try_lock():
                pass
            try:
                register_address = bytes([0x01])
                i2cbus.writeto(i2caddress_3, register_address)

                result_buffer = bytearray(2)
                i2cbus.readfrom_into(i2caddress_3, result_buffer)
                count = int.from_bytes(result_buffer, byteorder='little')
                print(count)

                register_address = bytes([0x02])
                result_buffer = bytearray(4)
                for i in range(count):
                    i2cbus.writeto(i2caddress_3, register_address)
                    i2cbus.readfrom_into(i2caddress_3, result_buffer)
                    data = int.from_bytes(result_buffer, byteorder='little')
                    print(data)
                    sensorQueue.put("GC11," + str(data))

                register_address = bytes([0x03])
                i2cbus.writeto(i2caddress_3, register_address)

                result_buffer = bytearray(2)
                i2cbus.readfrom_into(i2caddress_3, result_buffer)
                count = int.from_bytes(result_buffer, byteorder='little')
                print(count)

                register_address = bytes([0x04])
                result_buffer = bytearray(4)
                for i in range(count):
                    i2cbus.writeto(i2caddress_3, register_address)
                    i2cbus.readfrom_into(i2caddress_3, result_buffer)
                    data = int.from_bytes(result_buffer, byteorder='little')
                    print(data)
                    sensorQueue.put("GC12," + str(data))
            finally:
                i2cbus.unlock()

            #i2cbus.write_byte_data(i2caddress_3, 0x00, j)
            #byte_list = i2cbus.read_i2c_block_data(i2caddress_3, 0x00, 32)
            #char_array = "".join(chr(byte) for byte in result_buffer)
            #sensorQueue.put(char_array)

        if serialPort.is_open:
            serialPort.write(b'A')

            print(serialPort.read(94))
            bytesread = serialPort.readline()
            print(bytesread)
            elements = bytesread.decode().split(",")
            if ((len(elements) > 1) and elements[1] == "$GPGGA"):
                print ("Data: " + elements[2])
                print (elements)
                for item in elements:
                    print (str(item))
            elif (elements[0] != ""):
                print ("Command: " + str(elements[0][2:4]))
                command_in = int(elements[0][3:4])
                print (str(elements[0][2:4]))
            else:
                print ("Nothing")    
            for item in elements:
                print (str(item))

        if stop_input_thread:
           if serialPort.is_open:
                serialPort.close()
           break
        #sleep(3.0)
    
def output_worker_thread():
    while True:
        #print("Output\n")
        global stop_output_thread
        global sensorQueue
        global haspDatabase

        if serialPort.is_open:
            serialPort.write(b'B')

        #serialPort.write(ba)
        #if (serialPort.is_open):
        #    serialPort.write(randint())

        if stop_output_thread:
            if serialPort.is_open:
                serialPort.close()
            break
        #sleep(1.0)
    
def processing_worker_thread():
    while True:
        #print("Processing\n")
        global command_in
        global stop_processing_thread

        if serialPort.is_open:
            serialPort.write(b'C')

        # Dequeue sensor data and add them to the database
        #with sqlite3.connect("C:\\HASP\\HASP2026\\HASP2026\\HASP2026.sqlite3") as haspDatabase:
        with sqlite3.connect("/home/pi5/HASP2026/HASP2026.sqlite3") as haspDatabase:
            while not sensorQueue.empty():
                sData = sensorQueue.get()
                data_list = sData.split(',')
                #print(sData)

                # Add the data in the database
                #with sqlite3.connect("/home/pi5/HASP2026/HASP2026.sqlite3") as haspDatabase:
                cursor = haspDatabase.cursor()
                sqlStatement = "INSERT INTO TestTable (SensorID, Data) VALUES (?, ?)"
                cursor.execute(sqlStatement, (data_list[0], data_list[1],))

        if stop_processing_thread:
            if serialPort.is_open:
                serialPort.close()
            break
        #sleep(1.0)

# Initialize the startup conditions
print("Initializing")

# Initialize the serial port
#serialPort = serial.Serial(port="COM1", baudrate=9600, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE, parity='N')
serialPort = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE, parity='N')
serialPort.close()

modeControl = ModeControl()
command_in = 0
sensorQueue = queue.Queue(maxsize=10)

# Setup the database
#haspDatabase = sqlite3.connect("C:\\HASP\\HASP2026\\HASP2026\\HASP2026.sqlite3")
with sqlite3.connect("/home/pi5/HASP2026/HASP2026.sqlite3") as haspDatabase:
    cursor = haspDatabase.cursor()
    sqlStatement = "DROP TABLE IF EXISTS TestTable"
    cursor.execute(sqlStatement)
    sqlStatement = "CREATE TABLE IF NOT EXISTS TestTable (ID INTEGER PRIMARY KEY NOT NULL, SensorID TEXT(10), Data TEXT(48))"
    cursor.execute(sqlStatement)

# Setup the I2C communication with prepherals
i2cbus = I2C(1)
i2caddress_1 = 0x2A
i2caddress_2 = 0x2B
i2caddress_3 = 0x2C

# Setup the worker threads
input_thread = threading.Thread(target=input_worker_thread, args=())
processing_thraed = threading.Thread(target=processing_worker_thread, args=())
output_thread = threading.Thread(target=output_worker_thread, args=())

global stop_input_thread
global stop_output_thread
global stop_processing_thread

stop_input_thread = False
stop_output_thread = False
stop_processing_thread = False

print("Starting")    
input_thread.start()
processing_thraed.start()
output_thread.start()

while True:
    modeControl.SystemMCL()
    
    #key = getch()   # This is a blocking call
    #print(key)
    #user_input = input()
    #print(user_input)

    inputchar = sys.stdin.read(1)
    if inputchar == "q":
        sleep(1)
        if serialPort.is_open:
            serialPort.close()
        stop_input_thread = True
        stop_output_thread = True
        stop_processing_thread = True
        break
    
print("Done!")
