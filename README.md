# WORK IN PROGRESS

## Usage
#### Upload code onto sensor interface board
The PCB design files for the sensor interface board are in the `sensor_interface_board` directory. Once the board is populated with the components, please refer to the [Readme](sensor_interface/README.md) in the `sensor_interface` directory for instructions on connecting this to a processor board (I use a [NanoPi Duo](http://wiki.friendlyarm.com/wiki/index.php/NanoPi_Duo)).
Please refer to the same [Readme](sensor_interface/README.md) for instructions on uploading the C code onto the board.

Uploading the program onto the board connected to a processor board should be as simple as follows(assuming all dependencies have been installed as indicated in the [Readme](sensor_interface/README.md) file)(also assuming connection is made over `/dev/ttyS1` serial interface)
```
cd sensor_interface
make -j
USB_DEVICE=/dev/ttyS1 make upload
```
For any issues please refer [Readme](sensor_interface/README.md)

#### Running the positioning program
By default, the program publishes the pose over ROS. Make sure a ROS master is properly configured (ideally to a remote computer, to be able to visualize the pose and transforms on RViz). Run the following command in the root directory
```
python run.py
```

To record the pose data while running, use following flag(*data* is the filename to save pose data in)
```
python run.py -r data
```

To view the saved data, use the following command (Note: If running the plot command on the device over a remote SSH connection, make sure the `-X` flag is enabled to view the plot)
```
python run.py -plot data.npy
```

If the connected sensor interface board is on a different dev port (say /dev/ttyUSB0), use the following flag
```
python run.py -p /dev/ttyUSB0
```

For more options to run the program look up help
```
python run.py -h
```
