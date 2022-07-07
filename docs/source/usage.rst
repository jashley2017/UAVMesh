Usage
=====
This section shows some procedures and commands to work with this system 
in different contexts.

Procedure for System Integration 
--------------------------------

For the system to be fully operational, each unit should have 4 core components connected:

* Raspberry Pi4 + UAVMesh microSD card 
* Xbee DigiMesh Radio
* Ublox GPS (with PPS connected to GPIO26)
* DC-to-DC converter supporting at least 5V/4A 

A microSD card flashed with the UAVMesh custom OS will automatically log/transmit all 
sensors it can detect on startup. Please use the 
`Raspberry Pi Image Utility <https://www.raspberrypi.com/news/raspberry-pi-imager-imaging-utility/>`_ 
to flash the custom OS.

Commands for Developers
----------------------

Before working in the code, you should always source the environment so that the installed packages can be found.

.. code-block:: bash

   source install/setup.bash
   sudo systemctl stop roslaunch.service; # shut down the service while developing

To rebuild after making code changes...

.. code-block:: bash

  colcon build; # --packages-select $MY_PACKAGE <- for compiling only specific packages

To run specific sensors standalone (you can find the executable name in the setup.py of the node package)

.. code-block:: bash

   ros2 run $RUN_MY_NODE

To launch all sensors in testing. 

.. code-block:: bash

   ros2 launch plane main.launch.py 

If *any* changes to the devices folder take place (i.e. new device rules or service file edits) the files need to be recopied to their respective locations.

.. code-block:: bash 

   sudo cp 66-ftdi.rules /etc/udev/rules.d/
   sudo cp roslaunch.service /etc/systemd/system/

When you are finished, you might want to create a new image to be able to flash. Here is an example of the command on a linux machine but **be careful** 
mistaking drives and directions can destroy a lot of things. 

.. code-block:: bash 

  sudo dd if=/dev/sdd of=[mount point]/myimg.img bs=1M count=10000

For a more descriptive guide of how to do this (and even shrink the image!) see this `tomshardware guide <https://www.tomshardware.com/how-to/back-up-raspberry-pi-as-disk-image>`_.
