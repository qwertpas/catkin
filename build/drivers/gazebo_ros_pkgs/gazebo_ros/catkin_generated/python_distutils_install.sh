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

echo_and_run cd "/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ur3/Documents/catkin_AB6/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ur3/Documents/catkin_AB6/install/lib/python3/dist-packages:/home/ur3/Documents/catkin_AB6/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ur3/Documents/catkin_AB6/build" \
    "/usr/bin/python3" \
    "/home/ur3/Documents/catkin_AB6/src/drivers/gazebo_ros_pkgs/gazebo_ros/setup.py" \
     \
    build --build-base "/home/ur3/Documents/catkin_AB6/build/drivers/gazebo_ros_pkgs/gazebo_ros" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ur3/Documents/catkin_AB6/install" --install-scripts="/home/ur3/Documents/catkin_AB6/install/bin"
