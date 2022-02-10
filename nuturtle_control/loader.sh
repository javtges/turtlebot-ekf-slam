#!/bin/sh
export ROS_MASTER_URI=http://javtges:11311
. /home/msr/install/setup.sh
exec "$@"