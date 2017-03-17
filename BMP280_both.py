#!/usr/bin/python
# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# BMP280
# This code is designed to work with the BMP280_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/content/Barometer?sku=BMP280_I2CSs#tabs-0-product_tabset-2


import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

# Read data back from 0x88(136), 24 bytes
b1 = bus.read_i2c_block_data(0x76, 0x88, 24)# BMP280 address, 0x76(118)
b2 = bus.read_i2c_block_data(0x77, 0x88, 24)# BMP280 address, 0x77(118)

def convertDataTemp(b1):
    # Convert the data
    # Temp coefficents
    dig_T1 = b1[1] * 256 + b1[0]
    dig_T2 = b1[3] * 256 + b1[2]
    if dig_T2 > 32767 :
        dig_T2 -= 65536
    dig_T3 = b1[5] * 256 + b1[4]
    if dig_T3 > 32767 :
        dig_T3 -= 65536
    return dig_T1, dig_T2, dig_T3

def convertDataPressure(b1):
    # Pressure coefficents
    dig_P1 = b1[7] * 256 + b1[6]
    dig_P2 = b1[9] * 256 + b1[8]
    if dig_P2 > 32767 :
        dig_P2 -= 65536
    dig_P3 = b1[11] * 256 + b1[10]
    if dig_P3 > 32767 :
        dig_P3 -= 65536
    dig_P4 = b1[13] * 256 + b1[12]
    if dig_P4 > 32767 :
        dig_P4 -= 65536
    dig_P5 = b1[15] * 256 + b1[14]
    if dig_P5 > 32767 :
        dig_P5 -= 65536
    dig_P6 = b1[17] * 256 + b1[16]
    if dig_P6 > 32767 :
        dig_P6 -= 65536
    dig_P7 = b1[19] * 256 + b1[18]
    if dig_P7 > 32767 :
        dig_P7 -= 65536
    dig_P8 = b1[21] * 256 + b1[20]
    if dig_P8 > 32767 :
        dig_P8 -= 65536
    dig_P9 = b1[23] * 256 + b1[22]
    if dig_P9 > 32767 :
        dig_P9 -= 65536
    return dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9

def convertBitsNOffsets(data, dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9):
    # Convert pressure and temperature data to 19-bits
    adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
    adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16

    # Temperature offset calculations
    var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
    var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
    t_fine = (var1 + var2)
    cTemp = (var1 + var2) / 5120.0
    fTemp = cTemp * 1.8 + 32

    # Pressure offset calculations
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * (dig_P6) / 32768.0
    var2 = var2 + var1 * (dig_P5) * 2.0
    var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
    var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * (dig_P1)
    p = 1048576.0 - adc_p
    p = (p - (var2 / 4096.0)) * 6250.0 / var1
    var1 = (dig_P9) * p * p / 2147483648.0
    var2 = p * (dig_P8) / 32768.0
    pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100
    return pressure

tic = time.time() #start time

while True:
    dig_T1a, dig_T2a, dig_T3a = convertDataTemp(b1) 
    dig_T1b, dig_T2b, dig_T3b = convertDataTemp(b2) 
    dig_P1a, dig_P2a, dig_P3a, dig_P4a, dig_P5a, dig_P6a, dig_P7a, dig_P8a, dig_P9a = convertDataPressure(b1) 
    dig_P1b, dig_P2b, dig_P3b, dig_P4b, dig_P5b, dig_P6b, dig_P7b, dig_P8b, dig_P9b = convertDataPressure(b2) 

    # BMP280 address, 0x76(118)
    # Select Control measurement register, 0xF4(244)
    #		0x27(39)	Pressure and Temperature Oversampling rate = 1
    #					Normal mode
    bus.write_byte_data(0x76, 0xF4, 0x27)
    bus.write_byte_data(0x77, 0xF4, 0x27)
    # BMP280 address, 0x76(118)
    # Select Configuration register, 0xF5(245)
    #		0xA0(00)	Stand_by time = 1000 ms
    bus.write_byte_data(0x76, 0xF5, 0xA0)
    bus.write_byte_data(0x77, 0xF5, 0xA0)

    time.sleep(0.1)

    # BMP280 address, 0x76(118)
    # Read data back from 0xF7(247), 8 bytes
    # Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
    # Temperature xLSB, Humidity MSB, Humidity LSB
    data1 = bus.read_i2c_block_data(0x76, 0xF7, 8)
    data2 = bus.read_i2c_block_data(0x77, 0xF7, 8)

    p1 = convertBitsNOffsets(data1, dig_T1a, dig_T2a, dig_T3a, dig_P1a, dig_P2a, dig_P3a, dig_P4a, dig_P5a, dig_P6a, dig_P7a, dig_P8a, dig_P9a)
    p2 = convertBitsNOffsets(data2, dig_T1b, dig_T2b, dig_T3b, dig_P1b, dig_P2b, dig_P3b, dig_P4b, dig_P5b, dig_P6b, dig_P7b, dig_P8b, dig_P9b)

    print "Sensor 1 Pressure: %.4f hPa " %p1
    print "Sensor 2 Pressure: %.4f hPa " %p2
    toc = time.time()

     #---------------------CALCULATING FLOWRATE ----------------------
    import math 
    flowrate_list =[]

    delta_p = abs(p2-p1)*10
    print "Change in pressure: %.4f Pa" %delta_p
    density = 1.15 #density of air @ 35C
    d1 = 0.028
    d2 = 0.0105 
    a1 = (d1/2)**2*math.pi
    a2 = (d2/2)**2*math.pi
    v1 = math.sqrt((2*delta_p)/(density*(((a1/a2)**2)-1)))
    #print "initial velocity: %.4f m/s" %v1
    f1 = v1*a1 # intial flowrate 
    flowrate_list.append(f1)
    print "flowrate: %.4f m^3/s" %f1
    time_elapsed = time.time() - tic
    print "time: %.3f s" %time_elapsed
    print flowrate_list


