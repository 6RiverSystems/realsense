#!/bin/bash
source ~/.bashrc

exec ~/ros/set-affinity.sh &

echo 'Starting ros navigation stack'
runChuck
