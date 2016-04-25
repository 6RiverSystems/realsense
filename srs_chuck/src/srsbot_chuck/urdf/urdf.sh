#!/bin/bash
rosrun xacro xacro.py chuck.urdf.xacro > chuck.urdf && 
check_urdf chuck.urdf &&
urdf_to_graphiz chuck.urdf
