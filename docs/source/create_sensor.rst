Creating a simple sensor.
===========================

UAVMesh works very similarly to how standard ROS2 does when creating simple publishers and subscribers. 
In this case, a 'sensor' is considered a device that we would like to receive information from and send 
that information across the system. This means that the purpose of a sensor node is to be a publisher of 
that sensor's information.

Creating the sensor node.
-------------------------
A script has been provided, `create_sensor.sh <https://github.com/jashley2017/UAVMesh/create_sensor.sh>`_, 
that will automatically populate a sensor node from a template. Here is an example of its usage in creating a 
sensor called 'sonic_anemometer'

.. code-block:: bash
  :caption: Create sensor script

  ./create_sensor.sh sonic_anemometer

This will create the following heirarchy we can now work in.
| src/sonic_anemometer
| ├── sonic_anemometer
| |   ├── sonic_anemometer_node.py
| |   └── __init__.py
| ├── package.xml
| ├── setup.cfg
| ├── setup.py
| ├── resource
| └── test

The key files are, sonic_anemometer_node.py, package.xml, and setup.py. Here are each of their function. 

sonic_anemometer.py
  This is where the publisher code for consuming the sensor information lives. It will be populated with a simple publisher template.
package.xml
  This is where licensing information and package dependency lives. If your sensor uses other packages or python libraries they should be specified here as an 'exec_depend'.
setup.py 
  This file tells the ROS2 package manager where your source files are and how to build them. Likely no changes need to be made to this file, but take not of the name of your executable (you can change it).

Writing your sensor node script
-------------------------------
Looking inside the node script that you generated in the previous part.

.. code-block:: python
  :caption: Full python sensor node template 

  import serial
  import time
  import threading
  import rclpy
  from rclpy.node import Node
  from networked_sensor.networked_sensor import Sensor
  # TODO: your message type will look something like this import
  # from std_msgs.msg import String

  class SonicAnemometer(Sensor):
      MSG = 'TODO'
      TOPIC = 'TODO'

      def __init__(self):
        super().__init__('sonic_anemometer') # Sensor('sonic_anemometer')
        # publisher
        self.sensor_pub = self.create_publisher(self.MSG, self.TOPIC, 10)
        # example parameters, delete if you do not use
        self.declare_parameter('sonic_anemometer_port', '')
        self.declare_parameter('sonic_anemomter_baudrate', '')
        serial_dev = serial.Serial(self.get_parameter('sonic_anemometer_port').value, self.get_parameter('sonic_anemomter_baudrate').value)
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
            # TODO: create code that turns raw into sensor_msg
            sensor_msg = 'TODO'
            self.sensor_pub.publish(sensor_msg)

This is quite a bit of code so let's break it down incrementally, starting with the __init__ block.

.. code-block:: python

  self.sensor_pub = self.create_publisher(self.MSG, self.TOPIC, 10)

Here we are creating a ROS2 publisher that is defined to publish a ROS2 message of type 'self.MSG' over the topic 'self.TOPIC'. 
These two values will need to be determined by you and put where there are 'TODO's at the top of the class. In short, the message 
type determines the format of data that you can publish and the topic determines which pipe the message will go through upon 
publication.

.. code-block:: python

  # example parameters, delete if you do not use
  self.declare_parameter('sonic_anemometer_port', '')
  self.declare_parameter('sonic_anemomter_baudrate', '')
  serial_dev = serial.Serial(self.get_parameter('sonic_anemometer_port').value, self.get_parameter('sonic_anemomter_baudrate').value)

This part of the code is me taking an educated guess that your sensor is some sort of serial device you would like to connect to, usually USB. 
If that is the case then this part of the code is for you! Otherwise, you need to research tutorials of how your device interfaces with the 
computer and how to configure it in python. The 'declare_parameter' statements tell ROS2 to look for these parameters named 'sonic_anemometer_port' 
and 'sonic_anemomter_baudrate' at launch time. The second parameter in the statement is the default value if ROS2 doesn't find this. It is advisable 
to populate this with your best guess. Using these parameters, 'serial_dev' creates a pyserial device we can interface with later. 

.. code-block:: python

  # start the main logging loop
  self.read_thread = threading.Thread(target=self.sensor_loop, args=(serial_dev,))
  self.running = True
  self.read_thread.start()

In the final part of the '__init__' we want to setup a loop to continuously track the status of our serial device. ROS2 nodes commonly work like arduino's 
with a 'setup' in '__init__' and a 'loop' in your defined thread.

.. code-block:: python

  def sensor_loop(self, serial_dev):
      while rclpy.ok() and self.running:
          try:
              raw = serial_dev.readline().decode("ascii")
              sample_time = time.time() # nearest millisecond to sample
          except UnicodeDecodeError as e:
              self.get_logger().warning("got decode error, if this continues frequently restart program.")
              continue

Now that our '__init__' has started running our 'sensor_loop' let's take a look at what is going on there. Firstly, if the ROS2 master shuts down we want 
all related processes to finish as well, which is the purpose of the while statement including 'rclpy.ok()'. Second, 'serial_dev.readline()' reads the raw 
information coming from the sensor as an ascii string. We also take the time of the sample here too.

.. code-block:: python

  # TODO: create code that turns raw into sensor_msg
  sensor_msg = 'TODO'
  self.sensor_pub.publish(sensor_msg)

Finally, this part is where you will be adding the most code. Here the program needs to parse whatever raw information is 
coming from the sensor and form it into your chosen ROS2 message. The process is fairly simple and usually involves spliting 
the string and assigning each component to an attribute of your message object, then publishing. Don't forget to correctly 
type each attribute though!

Finding your USB device
-----------------------
Often times these projects require a multitude of sensors being logged simultaneously on the same device. The sensors can be 
the same or different or a mix of both. Therefore, there needs to be a way to identify and enumerate the sensors as USB devices. 
Luckily Linux provides a tool called UDEV for exactly this purpose. Unluckily, this does not mean it is a simple process to use 
UDEV. Before looking for yourself, check to see if the sensor manufacturer or someone online has already made device rules for 
your sensor. If you need to create your own udev rule, `here is a helpful tutorial <https://opensource.com/article/18/11/udev>`_. 
On the udevadm info command primarily look for the *idProduct* and *idVendor* attributes.

Once you have discovered the attributes that uniquely identify your device two things need to be done. Firstly, add a symlink 
command to the end of the rule to specify what your devices would like to be named. For instance 'SYMLINK+="xbee%n"' will create 
devices that look like '/dev/xbee1', '/dev/xbee2', etc. Then we need to add the udev rule to devices/66-ftdi.rules.

.. code-block:: bash

  echo $NEW_UDEV_RULE >> devices/66-ftdi.rules
  sudo cp devices/66-ftdi.rules /etc/udev/rules.d/66-ftdi.rules

Creating a launch specification
-------------------------------


