#!/bin/bash

catkin build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --
jq -s add build/**/compile_commands.json > compile_commands.json