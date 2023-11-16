#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Temperature, FluidPressure
import smbus2
from bmp280 import BMP280

def read_bmp280_data():
    # Initialize the I2C bus
    bus = smbus2.SMBus(2)  # Use 1 for LattePanda 3 Delta

    # Initialize the BMP280 sensor
    bmp280 = BMP280(i2c_dev=bus)

    # Create ROS publishers for temperature and pressure
    temperature_pub = rospy.Publisher('/bmp280/temperature', Temperature, queue_size=10)
    pressure_pub = rospy.Publisher('/bmp280/pressure', FluidPressure, queue_size=10)

    rospy.init_node('bmp280_reader', anonymous=True)

    rate = rospy.Rate(50)  # 1 Hz

    while not rospy.is_shutdown():
        # Read temperature and pressure
        temperature = bmp280.get_temperature()
        pressure = bmp280.get_pressure()

        # Create Temperature and FluidPressure messages
        temp_msg = Temperature()
        temp_msg.header.stamp = rospy.Time.now()
        temp_msg.temperature = temperature

        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = rospy.Time.now()
        pressure_msg.fluid_pressure = pressure

        # Publish the data
        temperature_pub.publish(temp_msg)
        pressure_pub.publish(pressure_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        read_bmp280_data()
    except rospy.ROSInterruptException:
        pass