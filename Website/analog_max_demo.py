#
# AnalogMax: All Sensors Demo
#

import serial
import json
from time import time, sleep
from matplotlib import pyplot as plt
import numpy as np

#
# Please change here to proper COM port name
#
ser = serial.Serial('/dev/ttyUSB0', 115200)

#
# DHT11 access functions
#
def DHT11_read_reg16(reg):
    ser.reset_input_buffer()
    cmd = "%x" % reg
    ser.write(bytearray(cmd,'utf8'))   # Send read register command
    s = ser.read(4)                    # Read two data bytes
    x = int(s,16)                      # Convert hex to int
    return x

#
# ADT7320 register access functions
#
def ADT7320_read_reg8(reg):
    ser.reset_input_buffer()
    cmd = "<%0.2x..>" % (reg*8 + 64)  #
    ser.write(bytearray(cmd,'utf8'))  # Send read register command
    s = ser.read(2)                   # Read data byte
    return int(s,16)                  # return unsigned 8 bit int

def ADT7320_read_reg16(reg):
    ser.reset_input_buffer()
    cmd = "<%0.2x....>" % (reg*8 + 64) #
    ser.write(bytearray(cmd,'utf8'))   # Send read register command
    s = ser.read(4)                    # Read two data bytes
    x = int(s,16)                      # Convert hex to int
    if x > 32768: x = x - 65536        # Convert to signed int
    return x

#
# ADPD register access functions
#
def ADPD_write_reg(reg, value):
    ser.reset_input_buffer() #
    cmd = "<c8K%0.2xK%0.2xK%0.2xK>" % (reg, (value >> 8), (value & 0xFF))  #
    ser.write(bytearray(cmd,'utf8'))  # Send command, REG and value

def ADPD_read_reg(reg):
    ser.reset_input_buffer() #
    cmd = "<c8K%0.2xK<c9K..m..M>" % reg  #
    ser.write(bytearray(cmd,'utf8'))  # Send command, REG and value
    s = ser.read(4)
    return int(s, 16)

def ADPD_read_fifo():
    fifolevel = ADPD_read_reg(0x00) >> 8
    # Do something if fifo level < 2 ?
    retval = ADPD_read_reg(0x60)
    retval = 1.0 * retval/16384
    return retval

#
# ADPD register init from ADI Wavetool Application
# Filename: ADPD188BI_SK.dcfg
#
def ADPD_init_SK():
    ADPD_write_reg(0x00 ,0x1000)
    ADPD_write_reg(0x01 ,0xC0FF)
    ADPD_write_reg(0x02 ,0x0005)
    ADPD_write_reg(0x06 ,0x0F00)
    ADPD_write_reg(0x09 ,0x00C8)
    ADPD_write_reg(0x10 ,0x0000)
    ADPD_write_reg(0x11 ,0x30A9)
    ADPD_write_reg(0x12 ,0x0050)
    ADPD_write_reg(0x14 ,0x0117)
    ADPD_write_reg(0x15 ,0x0220)
    ADPD_write_reg(0x18 ,0x1F00)
    ADPD_write_reg(0x19 ,0x3FFF)
    ADPD_write_reg(0x1A ,0x3FFF)
    ADPD_write_reg(0x1B ,0x3FFF)
    ADPD_write_reg(0x1E ,0x1F00)
    ADPD_write_reg(0x1F ,0x3FFF)
    ADPD_write_reg(0x20 ,0x3FFF)
    ADPD_write_reg(0x21 ,0x3FFF)
    ADPD_write_reg(0x22 ,0x3531)
    ADPD_write_reg(0x23 ,0x3533)
    ADPD_write_reg(0x24 ,0x3531)
    ADPD_write_reg(0x25 ,0x6317)
    ADPD_write_reg(0x30 ,0x0319)
    ADPD_write_reg(0x31 ,0x0810)
    ADPD_write_reg(0x34 ,0x0000)
    ADPD_write_reg(0x35 ,0x0319)
    ADPD_write_reg(0x36 ,0x0810)
    ADPD_write_reg(0x38 ,0x0000)
    ADPD_write_reg(0x39 ,0x2203)
    ADPD_write_reg(0x3B ,0x2203)
    ADPD_write_reg(0x3C ,0x31C6)
    ADPD_write_reg(0x3E ,0x0320)
    ADPD_write_reg(0x3F ,0x0320)
    ADPD_write_reg(0x42 ,0x1C34)
    ADPD_write_reg(0x43 ,0xADA5)
    ADPD_write_reg(0x44 ,0x1C34)
    ADPD_write_reg(0x45 ,0xADA5)
    ADPD_write_reg(0x4B ,0x269C)
    ADPD_write_reg(0x4D ,0x0082)
    ADPD_write_reg(0x54 ,0x0AA0)
    ADPD_write_reg(0x58 ,0x0000)
    ADPD_write_reg(0x59 ,0x0808)
    ADPD_write_reg(0x5A ,0x0010)
    ADPD_write_reg(0x5E ,0x0808)
    ADPD_write_reg(0x5F ,0x0000)

def ADPD_init():
    ADPD_init_SK()
    # Change one setting
    ADPD_write_reg(0x11 ,0x3065) # Channel A,B, Average, 16 bit sum of channels
    # Enter normal mode
    ADPD_write_reg(0x10 ,0x0002)

#
# ADXL362 register access functions
#
def ADXL362_write_reg(reg, value):
    cmd = "<0a%0.2x%0.2x>" % (reg, value) # Write command 0x0A
    ser.write(bytearray(cmd,'utf8'))  # Send command, REG and value

def ADXL362_read_reg8(reg):
    ser.reset_input_buffer()
    cmd = "<0b%0.2x..>" % reg # Read command 0x0B
    ser.write(bytearray(cmd,'utf8'))  # Send command, REG and value
    s = ser.read(2)
    return int(s,16)                  # Convert hex to int

def ADXL362_read_reg16(reg):
    ser.reset_input_buffer()
    cmd = "<0b%0.2x..><0b%0.2x..>" % (reg+1, reg) # Read command 0x0B
    ser.write(bytearray(cmd,'utf8'))  # Send command, REG and value
    s = ser.read(4)
    return int(s,16)                 # Convert hex to int

#
# Read sign extended 16 bit register
#
def ADXL362_read_reg16_sx(reg):
    t = ADXL362_read_reg16(reg)
    if t > 32768: t = t - 65536
    return t

def ADXL362_read_axis(axis):
    return ADXL362_read_reg16_sx(axis*2 + 0x0e)

def ADXL362_read_temperature():
    t = ADXL362_read_reg16_sx(0x14)
    return round((t-350)*0.065+25,2)

def get_all_sensors(data):
    measure_time = time()

    ser.flushInput()
    ser.reset_input_buffer()
    ser.write(b'[7]')    # Select DHT11 Channel
    dht11_humidity = float(DHT11_read_reg16(1)) / 256.0
    dht11_temperature = float(DHT11_read_reg16(0)) / 256.0
    ser.flushInput() # Flush junk

    ser.flushInput()
    ser.reset_input_buffer()
    ser.write(b'[1]')    # Select UHSA ADT Channel
    adt7320_temperature = float(ADT7320_read_reg16(2)) / 128.0
    ser.flushInput() # Flush junk

    ser.flushInput()
    ser.reset_input_buffer()
    ser.write(b'[6]')    # Select UHSA I2C Channel
    ser.reset_input_buffer()
    ADPD_read_reg(0x08) # Check sensor ID
    ADPD_init() # "Normal" Register Init
    ADPD_read_reg(0x00) # FIFO samples
    ADPD_read_reg(0x60) # FIFO data
    ser.flushInput()
    adpd_smoke_data_num = float(ADPD_read_fifo())
    adpd_smoke_data_den = float(ADPD_read_fifo())
    adpd_smoke_data_div = adpd_smoke_data_num / adpd_smoke_data_den
    ADPD_write_reg(0x0F, 0x0001) # Issue Softreset (Blue LED will go off, also the GPIO0 LED)
    ser.flushInput() # Flush junk

    ser.flushInput()
    ser.reset_input_buffer()
    ser.write(b'[0]')    # Select UHSA I2C Channel
    ADXL362_write_reg(0x2d, 0x02)     # Enable Measurement Mode
    ADXL362_read_axis(0) # Some dummy read first...
    accelerometer_x = float(ADXL362_read_axis(0)) * 4.0 / 4096.0
    accelerometer_y = float(ADXL362_read_axis(1)) * 4.0 / 4096.0
    accelerometer_z = float(ADXL362_read_axis(2)) * 4.0 / 4096.0
    ser.flushInput() # Flush junk

    data.append([measure_time, dht11_humidity, dht11_temperature, adt7320_temperature,
                 adpd_smoke_data_num, adpd_smoke_data_den, adpd_smoke_data_div,
                 accelerometer_x, accelerometer_y, accelerometer_z])

    print("Timestamp            :", measure_time)
    print("DHT11 Humidity       :", dht11_humidity)
    print("DHT11 Temperature    :", dht11_temperature)
    print("ADT7320 Temperature  :", adt7320_temperature)
    print("ADPD Smoke data      :", adpd_smoke_data_div)
    print("Accelerometer X Axis :", accelerometer_x)
    print("Accelerometer Y Axis :", accelerometer_y)
    print("Accelerometer Z Axis :", accelerometer_z)
    print("")


data = []
try:
    with open('/home/pi/Documents/AnalogMax/data.json') as data_file:
        data = json.load(data_file)
except:
    pass

max_elements = 60 * 24 * 7  # 7 days
if len(data) > max_elements:
    data = data[len(data) - max_elements:]

get_all_sensors(data)

with open('/home/pi/Documents/AnalogMax/data.json', 'w') as data_file:
    json.dump(data, data_file, indent=2)

print('Data saved in data.json file')

ser.close()

data_t = np.transpose(data)
timepoints = (data_t[0] - data_t[0][-1]) / (60.0 * 60.0)
data_t[3] = data_t[3] - 11.0
data_t[6] = data_t[6] - 1.0

fig0 = plt.figure()
fig0.suptitle('Humidity', fontsize='18', fontweight='bold')
plt.axes().grid(True)
plt.xlabel('Time [h]', fontsize='14', fontstyle='italic')
plt.ylabel('Humidity [%RH]', fontsize='14', fontstyle='italic')
plt.plot(timepoints, data_t[1])
plt.ylim([0.0, 75.1])
plt.legend(['External sensor'])
fig0.savefig('/var/www/html/images/humidity.png')

fig1 = plt.figure()
fig1.suptitle('Temperature', fontsize='18', fontweight='bold')
plt.axes().grid(True)
plt.xlabel('Time [h]', fontsize='14', fontstyle='italic')
plt.ylabel('Temperature [C]', fontsize='14', fontstyle='italic')
plt.plot(timepoints, data_t[2], timepoints, data_t[3])
plt.ylim([9.9, 30.1])
plt.legend(['External sensor', 'Internal sensor (cal.)'])
fig1.savefig('/var/www/html/images/temperature.png')

fig2 = plt.figure()
fig2.suptitle('Smoke sensor', fontsize='18', fontweight='bold')
plt.axes().grid(True)
plt.xlabel('Time [h]', fontsize='14', fontstyle='italic')
plt.ylabel('Sensor reading', fontsize='14', fontstyle='italic')
plt.plot(timepoints, data_t[6])
plt.ylim([-0.1, 1.1])
plt.legend(['Smoke detection (cal.)'])
fig2.savefig('/var/www/html/images/smoke.png')

fig3 = plt.figure()
fig3.suptitle('Accelerometer', fontsize='18', fontweight='bold')
plt.axes().grid(True)
plt.xlabel('Time [h]', fontsize='14', fontstyle='italic')
plt.ylabel('Accelerometer [g]', fontsize='14', fontstyle='italic')
plt.plot(timepoints, data_t[7], timepoints, data_t[8], timepoints, data_t[9])
plt.ylim([-1.1, 1.1])
plt.legend(['X Axis (cal.)', 'Y Axis (cal.)', 'Z Axis (cal.)'])
fig3.savefig('/var/www/html/images/accelerometer.png')

print('Images updated')
