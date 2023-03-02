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
$ cmake ..
$ make
```
This builds an executable called `Robokasi`, which can be run with the command `./Robokasi`.

