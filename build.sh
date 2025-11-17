#!/bin/bash

catkin_make_isolated -DCMAKE_EXPORT_COMPILE_COMMANDS=1 
jq -s add build_isolated/**/compile_commands.json > compile_commands.json