# Build

```
mkdir build

cd build

export PICO_SDK_PATH=../../pico-sdk

cmake ..

make
```

# File structure

.
├── micro_ros_raspberrypi_pico_sdk
├── mpu
│ ├── build
│ ├── CMakeLists.txt
│ ├── helper_3dmath.h
│ ├── I2Cdev.cpp
│ ├── I2Cdev.h
│ ├── MPU6050_6Axis_MotionApps_V6_12.h
│ ├── MPU6050.cpp
│ ├── mpu6050_DMP_port.cpp
│ ├── MPU6050.h
│ ├── pico_sdk_import.cmake
│ └── README.md
├── pico-examples
└── pico-sdk

# Data output

```
ypr: 4.772155,   -0.260317,      0.031157
```
