#!/bin/bash
sudo apt-get install git -y

# Connect to a wireless network
sudo nmcli d wifi connect <network> password <password> iface wlan0

# Install ROS

!!!!! NEW INSTRUCTIONS

http://wiki.ros.org/indigo/Installation/UbuntuARM

!!!!! OLD INSTRUCTIONS

sudo rm -rf /var/cache/apt/archives && sudo ln -s ~/.apt-cache /var/cache/apt/archives && mkdir -p ~/.apt-cache/partial
gpg --keyserver pgp.mit.edu --recv-keys 749D6EEC0353B12C
gpg --export --armor 749D6EEC0353B12C | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update -y
sudo apt-get install ros-indigo-desktop-full -y
sudo apt-get install ros-indigo-rgbd-launch -y
sudo apt-get install ros-indigo-depthimage-to-laserscan -y
sudo apt-get install ros-indigo-laser-filters -y
sudo apt-get install ros-indigo-navigation -y
sudo apt-get install ros-indigo-robot-model -y
sudo apt-get install ros-indigo-urdf -y
sudo apt-get install ros-indigo-xacro -y
sudo apt-get install ros-indigo-t2 -y
sudo apt-get install ros-indigo-tf2 -y
sudo apt-get install ros-indigo-tf2-sensor-msgs -y
sudo apt-get install ros-indigo-camera-info-manager -y
sudo apt-get install ros-indigo-joy -y
sudo apt-get install ros-indigo-rosbridge-server -y
sudo apt-get install ros-indigo-depthimage-to-laserscan -y
sudo apt-get install ros-indigo-robot-state-publisher -y

# Optional dev dependencies
rqt --force-discover

sudo rosdep init
rosdep update

# Change the password policy to accept short passwords in the file /etc/pam.d/common-password
# from:
# password [success=1 default=ignore] pam_unix.so obscure sha512
# to:
# password [success=1 default=ignore] pam_unix.so minlen=4 sha512

# Change the login background:
sudo -i
xhost +SI:localuser:lightdm
su lightdm -s /bin/bash
dconf-editor

# When the tool opens, navigate to com / canonical / unity-greeter, and change the background value to your
# custom image and disable both draw-grid and draw-user-backgrounds. Set the Idle time to 0.

# Change the hostname
sudo vim /etc/hostname
sudo vim /etc/hosts

# Disable the Unity ubuntu scrollbars:
echo export LIBOVERLAY_SCROLLBAR=0 | sudo tee -a /etc/X11/Xsession.d/99disable-overlay-scrollbars

# Install librealsense
git clone https://github.com/6RiverSystems/librealsense.git
cd ~/librealsense/
sudo apt-get install libusb-1.0-0-dev -y
./scripts/install_glfw3.sh
sudo cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo cp config/uvc.conf /etc/modprobe.d/
sudo udevadm control --reload-rules && udevadm trigger
cd ~/librealsense/scripts
./install-r200-udev-fix.sh 
# Optional (check if device is detected)
sudo dmesg | tail -n 1000
sudo rm /usr/lib/arm-linux-gnueabihf/libGL.so 
sudo ln -s /usr/lib/arm-linux-gnueabihf/tegra/libGL.so /usr/lib/arm-linux-gnueabihf/libGL.so
cd ~/librealsense
make BACKEND=LIBUVC
sudo make install

# Install other development dependencies
sudo apt-get install libsdl-image1.2-dev socat socat ccache python-gevent=1.0-1ubuntu1 libunwind8-dev expect-dev -y

echo "export PYTHONPATH=/usr/lib/python2.7/dist-packages:${PYTHONPATH}" >> ~/.bashrc
echo "export PATH=/usr/lib/ccache:$PATH" >> ~/.bashrc
echo "export ROS_PARALLEL_JOBS=-j8" >> ~/.bashrc
echo "source ~/.bashrc_ros" >> ~/.bashrc

echo "export ROBOT_NAME=dan-x1" >> ~/.bashrc_ros
echo "export ROS_MAP=6rhq" >> ~/.bashrc_ros
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc_ros

# Change all network interfaces to default to eth0 and wlan0
sudo rm /etc/udev/rules.d/70-persistent-net.rules

git clone https://github.com/6RiverSystems/ros.git ~/ros
sudo cp ~/ros/start-mfp-ros.sh /etc/init

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
sudo cp ~/mfp_bridge/start-mfp-node.sh /etc/init

sudo service mfp-bridge stop

# Update to point to correct warehouse map (if necessary)
# NOTE: Change production to barrett for the barrett map
vim /home/rivs/mfp_bridge/start-mfp-node.sh
NODE_ENV=production NODE_PATH=$NODE_PATH:. node app.js --id $ROBOT_NAME | tee log/bridge.log

cd ~/mfp_bridge
git pull
npm update
sudo service mfp-bridge start

git describe # Capture this version in the spreadsheet

# Install the graphical interface
git clone https://github.com/6RiverSystems/graphical_interface.git ~/graphical_interface

cd ~/graphical_interface

killall electron
killall electron

git pull
npm update

# Update to point to correct warehouse map (if necessary)
# NOTE: Change demo to barrett for the barrett map
export ENV=demo
npm run electron:clean
npm run electron:webpack
npm run electron:asar

sudo vim /etc/default/locale 
LANG=”en_US.UTF-8” 

GLLINK=-L/usr/lib/aarch64-linux-gnu/tegra make 

FILTER_OUT := 0_Simple/cdpSimplePrint/Makefile 0_Simple/cdpSimpleQuicksort/Makefile 2_Graphics/bindlessTexture/Makefile6_Advanced/cdpBezierTessellation/Makefile 6_Advanced/cdpQuadtree/Makefile 6_Advanced/cdpLUDecomposition/Makefile 6_Advanced/cdpAdvancedQuicksort/Makefile 7_CUDALibraries/simpleDevLibCUBLAS/Makefile 2_Graphics/simpleGLES_EGLOutput/Makefile 2_Graphics/simpleGLES/Makefile 5_Simulations/nbody_opengles/Makefile 5_Simulations/fluidsGLES/Makefile 0_Simple/simpleMPI/Makefile 

# Refresh the bash console:
source ~/.bashrc
