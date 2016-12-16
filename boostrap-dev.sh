#!/bin/bash
sudo apt-get install git -y

git clone https://github.com/6RiverSystems/ros.git ~/ros

# Instructions => ~/ros/bootstrap-dev.sh

# Set time to be eastern standard 12 hour clock (Date/Time Settings)

# Create rivs admin user

# Change rivs password to 6RS

# Connect to a wireless network
sudo nmcli d wifi connect <network> password <password> iface wlan0

############# ARM ROS #############
# Install ROS (http://wiki.ros.org/indigo/Installation/UbuntuARM)

sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

############# x86_64 ROS #############
gpg --keyserver pgp.mit.edu --recv-keys 749D6EEC0353B12C
gpg --export --armor 749D6EEC0353B12C | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

############# COMMON ROS #############
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116

sudo apt-get update

sudo apt-get install python-rosdep
sudo rosdep init
rosdep update

sudo apt-get install ros-indigo-ros-base ros-indigo-rgbd-launch ros-indigo-depthimage-to-laserscan ros-indigo-laser-filters ros-indigo-navigation ros-indigo-robot-model ros-indigo-urdf ros-indigo-urdf ros-indigo-xacro ros-indigo-tf ros-indigo-tf2 ros-indigo-tf2-sensor-msgs ros-indigo-camera-info-manager ros-indigo-joy ros-indigo-rosbridge-server ros-indigo-robot-state-publisher ros-indigo-pointcloud-to-laserscan ros-indigo-depthimage-to-laserscan -y

echo "export PYTHONPATH=/usr/lib/python2.7/dist-packages:${PYTHONPATH}" >> ~/.bashrc
echo "export PATH=/usr/lib/ccache:$PATH" >> ~/.bashrc
echo "export ROS_PARALLEL_JOBS=-j8" >> ~/.bashrc
echo "source ~/.bashrc_ros" >> ~/.bashrc

echo "export ROBOT_NAME=dan-x1" >> ~/.bashrc_ros
echo "export ROS_MAP=6rhq" >> ~/.bashrc_ros
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc_ros
echo "source ~/ros/.bashrc_ros_common" >> ~/.bashrc_ros

# Refresh the bash console:
source ~/.bashrc

# Optional dev dependencies
sudo apt-get install ros-indigo-rqt* ros-indigo-rviz -y
rqt --force-discover

sudo apt-get install python-rosdep
sudo rosdep init
rosdep update

# When the tool opens, navigate to com / canonical / unity-greeter, and change the background value to your
# custom image and disable both draw-grid and draw-user-backgrounds. Set the Idle time to 0.

# Change the hostname
sudo vim /etc/hostname
sudo vim /etc/hosts

# Install other development dependencies
sudo apt-get install libsdl-image1.2-dev socat socat ccache python-gevent=1.0-1ubuntu1 libunwind8-dev expect-dev network-manager-openvpn-gnome linux-firmware -y


# Install librealsense
git clone https://github.com/6RiverSystems/librealsense.git
cd ~/librealsense/
sudo apt-get install libusb-1.0-0-dev -y
./scripts/install_glfw3.sh
sudo cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo cp ./config/uvc.conf /etc/modprobe.d/
sudo udevadm control --reload-rules && udevadm trigger
cd ~/librealsense/scripts
./install-r200-udev-fix.sh 
cd ~/librealsense
make BACKEND=LIBUVC
sudo make install

# Optional (check if device is detected)
sudo dmesg | tail -n 1000
sudo rm /usr/lib/arm-linux-gnueabihf/libGL.so 
sudo ln -s /usr/lib/arm-linux-gnueabihf/tegra/libGL.so /usr/lib/arm-linux-gnueabihf/libGL.so


# Change all network interfaces to default to eth0 and wlan0
sudo rm /etc/udev/rules.d/70-persistent-net.rules

git clone https://github.com/6RiverSystems/ros.git ~/ros
sudo cp ~/ros/mfp-ros.conf /etc/init

sudo find /opt/ros -type f -exec sed -i 's/\.so\.2\.4\.8/.so/g' {} \;
sudo find /opt/ros -type f -exec sed -i 's/\/arm-linux-gnueabihf\/libopencv_/\/libopencv_/g' {} \;
sudo find /opt/ros -type f -exec sed -i 's/\/usr\/lib\/libopencv_ocl\.so;//g' {} \;

# Copy default logging config
cp ~/ros/.rosconsole.config ~/

# Install nodejs
sudo apt-get install curl -y
curl -sL https://deb.nodesource.com/setup_6.x | sudo -E bash -
sudo apt-get install -y nodejs

# Install the bridge
git clone https://github.com/6RiverSystems/mfp_bridge.git ~/mfp_bridge
sudo cp ~/mfp_bridge/mfp-bridge.conf /etc/init

# Update to point to correct warehouse map (if necessary)
# NOTE: Change production to barrett for the barrett map
vim /home/rivs/mfp_bridge/start-mfp-node.sh
NODE_ENV=production NODE_PATH=$NODE_PATH:. node app.js --id $ROBOT_NAME | tee log/bridge.log

cd ~/mfp_bridge
npm update

# Install the graphical interface
git clone https://github.com/6RiverSystems/graphical_interface.git ~/graphical_interface
cd ~/graphical_interface

sudo npm install -g electron-prebuilt

# Add /home/rivs/graphical_interface/start-gui.sh to the startup applications
# Launch and lock to task tray

npm update

# Update to point to correct warehouse map (if necessary)
# NOTE: Change demo to barrett for the barrett map
export ENV=demo
npm run electron:clean
npm run electron:webpack
npm run electron:asar

# Log into slack/google drive with rivs@6river.com

# Setup vpn => Download and extract rhq-openvpn.zip to home directory
# https://drive.google.com/a/6river.com/file/d/0BzfrCTr76JhbVXFLU0NHTDRGakk/view?usp=sharing

# Edit network settings and configure vpn (Add->Import VPN->Select rhq-openvpn\openvpn.ovpn)
# Set to start when wifi is connected (for remote facilities)

# Fix static IP for eth0
#   IP: 192.168.20.101
#   Mask: 255.255.255.0


# Disable unnecessary services
echo manual | sudo tee /etc/init/avahi-daemon.override
sudo apt-get remove cups
