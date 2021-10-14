#!/usr/bin/python3

import serial
import time
import sys
import os
import pynmea2
import RPi.GPIO as GPIO
from multiprocessing import Process, Value


# ros packages
import rclpy
from rclpy.node import Node
from environ_msgs.msg import NMEA_XDR, NMEA_MWV

BUTTON = 3
GPIO.setmode(GPIO.BCM) # GPIO3
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

AN1_PORT = '/dev/sonic1'
AN2_PORT = '/dev/sonic2'
AN3_PORT = '/dev/sonic3'
AN4_PORT = '/dev/sonic4'

LOG_HEADER = "Timestamp, Wind Speed/Temperature, Wind Angle, Reference, Status\n"

class Anemometer(Node):
    def __init__(self):
        super().__init__('anemometer')
        self.wind_pub = self.create_publisher(NMEA_MWV, 'wind_speed', 10)
        self.temp_pub = self.create_publisher(NMEA_XDR, 'air_temp', 10)

    def log_nmea(self, usb_dev, dev_num, outpath, button_pressed):
        with open(f"{outpath}{dev_num}.log", "w") as outfile:
            outfile.write(LOG_HEADER)
            sample_time = 0 # resolve scope issue
            while button_pressed.value != 1:
                try:
                    nmea_raw = usb_dev.readline().decode("ascii")
                    sample_time = int(round(time.time()*1000)) # nearest millisecond to sample
                except UnicodeDecodeError as e:
                    print("WARNING: got decode error, if this continues frequently restart program.")
                    continue
                if nmea_raw[0] == '$':
                    nmea_msg = pynmea2.parse(nmea_raw)
                    if (nmea_msg.sentence_type == "XDR"):
                        if(nmea_msg.id == "TempAir"):
                            outstring = f"{sample_time},{nmea_msg.value}{nmea_msg.units}\n"
                        else:
                            print(f"Got unknown NMEA XDR message: {nmea_msg.id}")
                            continue
                    elif (nmea_msg.sentence_type == "MWV"):
                        outstring = f"{sample_time},{nmea_msg.wind_speed}{nmea_msg.wind_speed_units},{nmea_msg.wind_angle},{nmea_msg.reference},{nmea_msg.status}\n"
                    else:
                        print(f"Got unknown NMEA message: {nmea_msg.sentence_type}")
                        continue
                    outfile.write(outstring)
                else:
                    print(f"Got unknown message")
        return None

    @staticmethod
    def create_windlog_dir():
        run_number = 0
        while os.path.isdir(f"/mnt/sda1/{run_number}"):
            run_number += 1
        os.mkdir(f"/mnt/sda1/{run_number}")
        return run_number

if __name__ == '__main__':
    print("Waiting for start button press")
    while GPIO.input(BUTTON) == GPIO.HIGH:
        time.sleep(0.1)
    print("Button Pressed, logging started")
    time.sleep(2)
    os.system('sudo python3 ./iMET_sampling.py /mnt/sda1/iMET_data &')

    run_number = create_windlog_dir()

    an1 = serial.Serial(AN1_PORT, 4800)
    an2 = serial.Serial(AN2_PORT, 4800)
    an3 = serial.Serial(AN3_PORT, 4800)
    an4 = serial.Serial(AN4_PORT, 4800)

    button_pressed = Value('i', 0) # this type allows for threadsafe process data sharing, nothing special, just an int.

    p1 = Process(target=log_nmea, args=(an1, 1, f"/mnt/sda1/{run_number}/windlog_dev", button_pressed))
    p2 = Process(target=log_nmea, args=(an2, 2, f"/mnt/sda1/{run_number}/windlog_dev", button_pressed))
    p3 = Process(target=log_nmea, args=(an3, 3, f"/mnt/sda1/{run_number}/windlog_dev", button_pressed))
    p4 = Process(target=log_nmea, args=(an4, 4, f"/mnt/sda1/{run_number}/windlog_dev", button_pressed))

    p1.start()
    p2.start()
    p3.start()
    p4.start()


    print("Awaiting stop button press")
    while GPIO.input(BUTTON) == GPIO.HIGH:
        time.sleep(0.1)
    print("Stop button pressed, wrapping up")
    button_pressed.value = 1
    os.system("sudo pkill -9 -f iMET_sampling.py")

    [p.join for p in [p1,p2 ,p3,p4]]
