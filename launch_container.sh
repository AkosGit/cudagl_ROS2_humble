#!/bin/sh

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run --gpus all --privileged --rm -it \
           --volume=$XSOCK:$XSOCK:rw \
           --volume=$XAUTH:$XAUTH:rw \
           --volume=$HOME:$HOME:rw \
           --volume=./workspace:/workspace \
           --shm-size=1gb \
           --env="XAUTHORITY=${XAUTH}" \
           --env="DISPLAY=${DISPLAY}" \
           --env=TERM=xterm-256color \
           --env=QT_X11_NO_MITSHM=1 \
           --name=ROS \
           atinfinity/cudagl:11.8.0-cudnn8-devel-ubuntu22.04 \
           bash

# fix for https://github.com/ros2/ros2/issues/1531
#docker exec -it ROS ulimit -n 1024
