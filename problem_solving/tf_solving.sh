sudo apt update
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules empy==3.2.2

mkdir -p ~/ploblem_solving/tf_solving/src; cd ~/ploblem_solving/tf_solving
catkin_make
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/NoppanutPat/geometry2.git -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r

catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so