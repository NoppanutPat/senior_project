sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-kinetic-cv-bridge

mkdir -p ~/ploblem_solving/cv_bridge_solving/src
cd ~/ploblem_solving/cv_bridge_solving
catkin init

catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

catkin config --install

git clone https://github.com/NoppanutPat/vision_opencv.git src/vision_opencv

apt-cache show ros-melodic-cv-bridge | grep Version

# cd src/vision_opencv/
# git checkout 1.12.8
# cd ../../
# catkin build cv_bridge
# source install/setup.bash --extend
