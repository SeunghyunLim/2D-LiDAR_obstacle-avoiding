# 2D-LiDAR Obstacle Avoiding
## Requirements & Envs
On ubuntu 18.04, ros-melodic
```
sudo apt-get install ros-melodic-hls-lfcd-lds-driver

sudo chmod a+rw /dev/ttyUSB0
roslaunch hls_lfcd_lds_driver hlds_laser.launch
```
## Smoothed direction ON

<center><img src="https://github.com/SeunghyunLim/2D-LiDAR/blob/main/gif/2dLiDAR_avoiding_smooth.gif" alt="drawing" width="980"/></center>

## Smoothed direction OFF

<center><img src="https://github.com/SeunghyunLim/2D-LiDAR/blob/main/gif/2dLiDAR_avoiding.gif" alt="drawing" width="980"/></center>

## Application on simulation(pybullet)

<center><img src="https://github.com/SeunghyunLim/2D-LiDAR/blob/main/gif/sim_avoiding.gif" alt="drawing" width="980"/></center>

## Application on real-world robot
<center><img src="https://github.com/SeunghyunLim/2D-LiDAR/blob/main/gif/real_avoiding.gif" alt="drawing" width="980"/></center>
