# gsu ros2 node
Groundstation unit node for data aggregation of a UAV system. This project communicates of an XBee radio network and synchronizes time on GPS using interpolation. 

# Important files
- gsu/run.py
  - Records data received from the Xbee to a log file.
- gsu/tsdb.py
  - Records data received to an influxdb database. Must have InfluxDB installed and configured to work. 
- launch/logger.launch.py
  - Launches the gps node and run.py to begin recording information. The gps time is used to measure time delay of each packet. 
- launch/tsdb.launch.py
  - Launches the gps node and tsdb.py to begin recording information into the database. The gps time is used to measure time delay of each packet. 
