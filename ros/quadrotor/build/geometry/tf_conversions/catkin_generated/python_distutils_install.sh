#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/pat/drone/senior_project/ros/quadrotor/src/geometry/tf_conversions"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/pat/drone/senior_project/ros/quadrotor/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/pat/drone/senior_project/ros/quadrotor/install/lib/python3/dist-packages:/home/pat/drone/senior_project/ros/quadrotor/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/pat/drone/senior_project/ros/quadrotor/build" \
    "/usr/bin/python3" \
    "/home/pat/drone/senior_project/ros/quadrotor/src/geometry/tf_conversions/setup.py" \
    egg_info --egg-base /home/pat/drone/senior_project/ros/quadrotor/build/geometry/tf_conversions \
    build --build-base "/home/pat/drone/senior_project/ros/quadrotor/build/geometry/tf_conversions" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/pat/drone/senior_project/ros/quadrotor/install" --install-scripts="/home/pat/drone/senior_project/ros/quadrotor/install/bin"
