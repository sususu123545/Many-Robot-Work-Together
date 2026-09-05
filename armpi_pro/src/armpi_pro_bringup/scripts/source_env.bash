#!/bin/zsh
setopt no_nomatch

sudo chgrp i2c /dev/i2c-*
sudo chgrp gpio /dev/gpio*

export HOST_IP=raspberrypi
export MASTER_IP=192.168.149.1
export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$HOST_IP

echo -e "ROS_HOSTNAME: \033[32m$ROS_HOSTNAME\033[0m"
echo -e "ROS_MASTER_URI: \033[32m$ROS_MASTER_URI\033[0m"

if [ $ZSH_VERSION ]; then
  . /opt/ros/noetic/setup.zsh
  . $HOME/armpi_pro/devel/setup.zsh
elif [ $BASH_VERSION ]; then
  . /opt/ros/noetic/setup.bash
  . $HOME/armpi_pro/devel/setup.bash
else
  . /opt/ros/noetic/setup.sh
  . $HOME/armpi_pro/devel/setup.sh
fi
export DISPLAY=:0.0
exec "$@"
