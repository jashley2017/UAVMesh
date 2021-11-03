#!path-to-python3 #TODO
import os
import time
import serial
import sys
from gpiozero import LED, Button

import rclpy
from rclpy.node import Node
from environ_msgs.msg import iMET_msg



class iMET(Node):
	def __init__(self):
		super().__init__('iMET')
		self.iMET_pub = self.create_publisher(IMET_msg, 'iMET_data', 10)
		self.declare_parameter('iMET_port', '/dev/iMET1')
		self.declare_parameter('iMET_baud', '9600')
		#TODO: double check baud, may need to add parity and stop bits arguments
		iMET1 = serial.Serial(self.get_parameter('iMET_port'), self.get_parameter('iMET_baud'))
		self.read_thread = threading.Thread(target=self.log_Imet, args=(iMET1,)) #TODO
		self.running = True
		self.read_thread.start
	
	def __del__(self):
		self.running = False
		self.read_thread.join()

	def log_iMET(self, usb_device):
		while self.running():
			try:
				iMET_raw = usb_dev.readline().decode("ascii")
				sample_time = time.time()

				parsed_iMET = pynmea2.parse(iMET_raw)
				ros_iMET_msg = IMET_msg()
				ros_iMET_msg.header.stamp.sec = int(sample_time)
				ros_iMET_msg.header.stamp.nsec = int((sample_time - int(sample_time))*1000000000)

				ros_iMET_msg.XQ = parsed_iMET.XQ
				ros_iMET_msg.pressure = parsed_iMET.pressure
				ros_iMET_msg.temp = parsed_iMET.temp
				ros_iMET_msg.rel_humidity = parsed_iMET.rel_humidity
				ros_iMET_msg.humidity_temp = parsed_iMET.humidity_temp
				ros_iMET_msg.date = parsed_iMET.date
				ros_iMET_msg.longitude = parsed_iMET.longitude
				ros_iMET_msg.latitude = parsed_iMET.latitude
				ros_iMET_msg.altitude = parsed_iMET.altitude
				ros_iMET_msg.num_sat = parsed_iMET.num_sat

				self.iMET_pub(ros_iMET_msg)

		return None

def main(args=None):
	print("launching")
	rclpy.init(args=args)
	iMET_pub = iMET()
	rclpy.spin(iMET_pub)
	iMET_pub.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

