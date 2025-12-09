#!/bin/bash

# CAUTION: This script is created from a localmachine configuration
# DO NOT RUN THIS DIRECTLY. Only use the commands inside as a reference
# on what is needed to be done.

# Requires a docker image bundled with urcap external-controller named ursim
# Create network ursim_net
sudo docker network create --subnet=192.168.56.0/24 ursim_net

# Launch the container
sudo docker run --rm --net ursim_net --ip 192.168.56.101 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 -e ROBOT_MODEL=UR10 -it ursim
