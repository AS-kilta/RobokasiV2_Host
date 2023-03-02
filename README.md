# RobokasiV2_Host

This project is generated from Lehdari's sdl_window project template by Lehdari on 2018-10-10 using Pulautin v0.01.

## Setup
### Install dependencies
The project depends on the following libraries:
* [SDL2](https://www.libsdl.org/)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [libserialport](https://sigrok.org/wiki/Libserialport)

#### Examples
**Ubuntu 20.04**

The required dependencies can be installed with the command

`$ sudo apt install -y cmake libsdl2-dev libeigen3-dev libserialport-dev`

## Build and run
To build the application run the following commands, assuming the current directory is the repository root
```
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ make
```
This builds an executable called `Robokasi`, which can be run with the command `./Robokasi`.

###Troubleshooting:

Window "Serial Config: Motor Drive" is empty.
- libserialport-dev is not properly installed/detected

When pressing connect a popup says "Connection failed"
1. Press the emergency stop button to reset the motor controller
2. Permission issue with serial ports, add current user to group dialout
- sudo usermod -a -G dialout $USER
- logout and login and try the program again


###Other info
Motor controller firmware and serial protocol
https://github.com/tkln/motor-controller-fw
