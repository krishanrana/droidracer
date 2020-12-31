__author__ = 'Geir Istad'
"""
MPU6050 Python I2C Class - MPU6050 example usage
Copyright (c) 2015 Geir Istad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from MPU6050 import MPU6050
import time

i2c_bus = 1
device_address = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = -481
y_accel_offset = 2130
z_accel_offset = 2863
x_gyro_offset = 105
y_gyro_offset = -43
z_gyro_offset = -39
enable_debug_output = True

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

mpu.dmp_initialize()

mpu.set_full_scale_accel_range(0x00) 
mpu.set_full_scale_gyro_range(0x00)
#98Hz low pass filter
mpu.set_DMP_enabled(True)
mpu_int_status = mpu.get_int_status()
print(hex(mpu_int_status))

packet_size = mpu.DMP_get_FIFO_packet_size()
print(packet_size)
FIFO_count = mpu.get_FIFO_count()
print(FIFO_count)

count = 0
FIFO_buffer = [0]*42
t0 = time.time()
FIFO_count_list = list()
while count < 10000:
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()

    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
        #print('overflow!')
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
        #t = time.time()
        #print(t-t0)
        #t0 = t   
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        accel = mpu.DMP_get_acceleration(FIFO_buffer)
        quat = mpu.DMP_get_quaternion(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        roll_pitch_yaw = mpu.DMP_get_roll_pitch_yaw(quat, grav)
        
        
        
        if count % 100 == 0:
            print('accel - x:' + str(accel.x))
            print('accel - y:' + str(accel.y))
            print('accel - z:' + str(accel.z))
            print('roll: ' + str(roll_pitch_yaw.x))
            print('pitch: ' + str(roll_pitch_yaw.y))
            print('yaw: ' + str(roll_pitch_yaw.z))
            print('gravity:' + str(grav.z))
            
        count += 1
