# Ouster LiDAR Low-Level API Demo

This project demonstrates how to use the Ouster LiDAR's low-level API.
The code has been tested under Ubuntu 22.04 with the Ouster OS1-64-Rev06 LiDAR sensor.

## Prerequisites

Before building the project, you need to install the following libraries:

### Install libcurl 

```bash
sudo apt update
sudo apt install -y libcurl4-openssl-dev
```

### Install OpenCV

```bash
sudo apt install -y libopencv-dev
```

### Install PDAL

```bash
sudo apt install -y pdal libpdal-dev
```

## Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```

