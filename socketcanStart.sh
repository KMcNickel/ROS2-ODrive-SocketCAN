#/bin/bash

ros2 lifecycle set /agv0/can0/receiver configure
ros2 lifecycle set /agv0/can0/sender configure
ros2 lifecycle set /agv0/can0/receiver activate
ros2 lifecycle set /agv0/can0/sender activate