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

echo_and_run cd "/home/hu/ProjectOtter/pararl_2/ROS/src/vision_opencv/image_geometry"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hu/ProjectOtter/pararl_2/ROS/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hu/ProjectOtter/pararl_2/ROS/install/lib/python3/dist-packages:/home/hu/ProjectOtter/pararl_2/ROS/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hu/ProjectOtter/pararl_2/ROS/build" \
    "/home/hu/.conda/envs/py35/bin/python" \
    "/home/hu/ProjectOtter/pararl_2/ROS/src/vision_opencv/image_geometry/setup.py" \
    build --build-base "/home/hu/ProjectOtter/pararl_2/ROS/build/vision_opencv/image_geometry" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/hu/ProjectOtter/pararl_2/ROS/install" --install-scripts="/home/hu/ProjectOtter/pararl_2/ROS/install/bin"
