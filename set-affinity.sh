#!/bin/bash

# Wait for move_base to start
until pids=$(pidof move_base)
do   
  echo 'Waiting for move_base to start...'
  sleep 1
done

# move_base has now started.

for pid in $pids
do
  echo 'Setting move_base ($pid) to run on CPU 4'

  taskset -cp 3 $pid
#  renice -n -5 -p $pid
done
