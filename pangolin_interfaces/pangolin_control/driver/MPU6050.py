"""This program handles the communication over I2C
between a Raspberry Pi and a MPU-6050 Gyroscope / Accelerometer combo.
Made by: MrTijn/Tijndagamer
Released under the MIT License
Copyright (c) 2015, 2016, 2017 MrTijn/Tijndagamer
"""
from smbus2 import SMBus
import struct#, math
from Kalman import *
import math
import os

class MPU6050Base:

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address=0x68, bus=1):
        self.address = address
        self.bus = SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

        self.set_accel_range(self.ACCEL_RANGE_2G)
        self.set_gyro_range(self.GYRO_RANGE_2000DEG)

        # self.SecondOrderFilterX = self._SecondOrderFilter()
        # self.SecondOrderFilterY = self._SecondOrderFilter()
        
    # I2C communication methods

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.

        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    # MPU-6050 Methods

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.

        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.

        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to.

        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g = False):
        """Gets and returns the X, Y and Z values from the accelerometer.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}


    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.

        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, raw = False):
        """Reads the range the gyroscope is set to.

        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.

        Returns the read values in a dictionary.
        """
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG  
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier

        return {'x': round(-y,2), 'y': round(x, 2), 'z': round(z, 2)}

    def get_all_data(self):
        """Reads and returns all the available data."""
        # data = self.bus.read_i2c_block_data(self.address,self.ACCEL_XOUT0,14)
        # data = struct.unpack('>hhhhhhh',bytes(data))

        # temp = (data[3] / 340.0) + 36.53

        # accel = data[:3]
        # accel = [a/self.ACCEL_SCALE_MODIFIER_2G*self.GRAVITIY_MS2 for a in accel]
        # gyro = data[-3:]
        # gyro = [g/self.GYRO_SCALE_MODIFIER_2000DEG for g in gyro]
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        return {"accel":accel, "gyro":gyro, "temp":temp}


class KalmanIMU:
    def __init__(self):
        self.Kp = 100 
        self.Ki = 0.002 
        self.halfT = 0.001 

        self.q0 = 1
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

        self.exInt = 0
        self.eyInt = 0
        self.ezInt = 0
        self.pitch = 0
        self.roll =0
        self.yaw = 0
        
        self.sensor = MPU6050Base(address=0x68,bus=1) 
        self.sensor.set_accel_range(self.sensor.ACCEL_RANGE_2G)   
        self.sensor.set_gyro_range(self.sensor.GYRO_RANGE_250DEG)  
        
        self.kalman_filter_AX =  Kalman_filter(0.001,0.1)
        self.kalman_filter_AY =  Kalman_filter(0.001,0.1)
        self.kalman_filter_AZ =  Kalman_filter(0.001,0.1)

        self.kalman_filter_GX =  Kalman_filter(0.001,0.1)
        self.kalman_filter_GY =  Kalman_filter(0.001,0.1)
        self.kalman_filter_GZ =  Kalman_filter(0.001,0.1)
        
        self.Error_value_accel_data,self.Error_value_gyro_data=self.average_filter()
    
    def average_filter(self):
        sum_accel_x=0
        sum_accel_y=0
        sum_accel_z=0
        
        sum_gyro_x=0
        sum_gyro_y=0
        sum_gyro_z=0
        
        for i in range(100):
            accel_data = self.sensor.get_accel_data()   
            gyro_data = self.sensor.get_gyro_data()      
            
            sum_accel_x+=accel_data['x']
            sum_accel_y+=accel_data['y']
            sum_accel_z+=accel_data['z']
            
            sum_gyro_x+=gyro_data['x']
            sum_gyro_y+=gyro_data['y']
            sum_gyro_z+=gyro_data['z']
            
        sum_accel_x/=100
        sum_accel_y/=100
        sum_accel_z/=100
        
        sum_gyro_x/=100
        sum_gyro_y/=100
        sum_gyro_z/=100
        
        accel_data['x']=sum_accel_x
        accel_data['y']=sum_accel_y
        accel_data['z']=sum_accel_z-9.8
        
        gyro_data['x']=sum_gyro_x
        gyro_data['y']=sum_gyro_y
        gyro_data['z']=sum_gyro_z
        return accel_data,gyro_data
    
    def getKalmanData(self):
        accel_data = self.sensor.get_accel_data()    
         
        gyro_data = self.sensor.get_gyro_data() 
        ax=self.kalman_filter_AX.kalman(accel_data['x']-self.Error_value_accel_data['x'])
        ay=self.kalman_filter_AY.kalman(accel_data['y']-self.Error_value_accel_data['y'])
        az=self.kalman_filter_AZ.kalman(accel_data['z']-self.Error_value_accel_data['z'])
        gx=self.kalman_filter_GX.kalman(gyro_data['x']-self.Error_value_gyro_data['x'])
        gy=self.kalman_filter_GY.kalman(gyro_data['y']-self.Error_value_gyro_data['y'])
        gz=self.kalman_filter_GZ.kalman(gyro_data['z']-self.Error_value_gyro_data['z'])

        norm = math.sqrt(ax*ax+ay*ay+az*az)
        
        ax = ax/norm
        ay = ay/norm
        az = az/norm
        
        vx = 2*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3
        
        ex = (ay*vz - az*vy)
        ey = (az*vx - ax*vz)
        ez = (ax*vy - ay*vx)
        
        self.exInt += ex*self.Ki
        self.eyInt += ey*self.Ki
        self.ezInt += ez*self.Ki
        
        gx += self.Kp*ex + self.exInt
        gy += self.Kp*ey + self.eyInt
        gz += self.Kp*ez + self.ezInt
        
        self.q0 += (-self.q1*gx - self.q2*gy - self.q3*gz)*self.halfT
        self.q1 += (self.q0*gx + self.q2*gz - self.q3*gy)*self.halfT
        self.q2 += (self.q0*gy - self.q1*gz + self.q3*gx)*self.halfT
        self.q3 += (self.q0*gz + self.q1*gy - self.q2*gx)*self.halfT
        
        norm = math.sqrt(self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3)
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm

        accel = {'x': ax, 'y': ay, 'z': az}
        gyro = {'x': gx, 'y': gy, 'z': gz}
        orientation = {'q0': self.q0, 'q1': self.q1, 'q2': self.q2, 'q3':self.q3}
        pitch = math.asin(-2*self.q1*self.q3+2*self.q0*self.q2)*57.3
        roll = math.atan2(2*self.q2*self.q3+2*self.q0*self.q1,-2*self.q1*self.q1-2*self.q2*self.q2+1)*57.3
        yaw = math.atan2(2*(self.q1*self.q2 + self.q0*self.q3),self.q0*self.q0+self.q1*self.q1-self.q2*self.q2-self.q3*self.q3)*57.3
        # self.pitch = pitch
        # self.roll =roll
        # self.yaw = yaw
        # return [self.q0,self.q1,self.q2,self.q3]
        # return self.roll , self.pitch, self.yaw
        # print(roll , pitch , yaw)
        return {"orientation":orientation, "accel":accel, "gyro":gyro}


if __name__ == "__main__":
    import time
    mpu_raw = MPU6050Base()
    mpu = KalmanIMU()
    time1=time.time()
    while True:
        time.sleep(0.05)
        # print(mpu_raw.get_gyro_data())
        accel = mpu.getKalmanData()
        # print(accel["gyro"])