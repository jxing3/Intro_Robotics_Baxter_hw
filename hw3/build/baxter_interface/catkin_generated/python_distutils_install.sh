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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jesse/ros_sandbox/hw3/src/baxter_interface"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jesse/ros_sandbox/hw3/install/lib/python2.7/dist-packages:/home/jesse/ros_sandbox/hw3/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jesse/ros_sandbox/hw3/build" \
    "/usr/bin/python" \
    "/home/jesse/ros_sandbox/hw3/src/baxter_interface/setup.py" \
    build --build-base "/home/jesse/ros_sandbox/hw3/build/baxter_interface" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/jesse/ros_sandbox/hw3/install" --install-scripts="/home/jesse/ros_sandbox/hw3/install/bin"
