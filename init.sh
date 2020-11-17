xterm -e "roscore" &
xterm -e "echo pwd | sudo -S chmod 777 /dev/ttyUSB0" &
xterm -e "sleep 12 && roslaunch hls_lfcd_lds_driver hlds_laser.launch" &
