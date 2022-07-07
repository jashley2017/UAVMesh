import serial
import time
import threading
import rclpy
from rclpy import Node
from networked_sensor.networked_sensor import Sensor
# TODO: your message type will look something like this import
# from std_msgs.msg import {{ msg_type }}

class {{camel_name}}(Sensor):
    MSG = {{ msg_type }}
    TOPIC = '{{ msg_topic }}'

    def __init__(self):
      super().__init__('{{snake_name}}_node') # Sensor('{{snake_name}}')
      # publisher
      self.sensor_pub = self.create_publisher(self.MSG, self.TOPIC, 10)
      # example parameters, delete if you do not use
      self.declare_parameter('{{snake_name}}_port', '')
      self.declare_parameter('{{snake_name}}_baudrate', '')
      serial_dev = serial.Serial(self.get_parameter('{{snake_name}}_port').value, self.get_parameter('{{snake_name}}_baudrate').value)
      # start the main logging loop
      self.read_thread = threading.Thread(target=self.sensor_loop, args=(serial_dev,))
      self.running = True
      self.read_thread.start()

  def __del__(self):
      self.running = False
      if self.read_thread:
          self.read_thread.join()

  def sensor_loop(self, serial_dev):
      while rclpy.ok() and self.running:
          try:
              raw = serial_dev.readline().decode("ascii")
              sample_time = time.time() # nearest millisecond to sample
          except UnicodeDecodeError as e:
              self.get_logger().warning("got decode error, if this continues frequently restart program.")
              continue
          sensor_msg = {{msg_type}}()
          # TODO: create code that turns raw into sensor_msg
          self.sensor_pub.publish(sensor_msg)

def main():
    rclpy.init(args=args)
    sensor_pub = {{camel_name}}()
    rclpy.spin(sensor_pub)
    sensor_pub.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
