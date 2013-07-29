#! /usr/bin/env sh

if [ -n "$ROS_PACKAGE_PATH" ]
then
  echo "Using ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
else
  echo ROS_PACKAGE_PATH not set
fi

PYYAML_DIR=$(rospack find morse_pyyaml)
MORSE_DIR=$(rospack find morse)
ROSLIB_DIR=$(rospack find roslib)
BLENDER_DIR=$(rospack find morse_blender)

# set python version for morse
PYTHON_VERSION=python3.2

if [ -z "$PYTHON3_EXECUTABLE" ]; then
    if which python3.2; then
        PYTHON3_EXECUTABLE=`which python3.2`
    else
        echo "Could not find python3 executable. Please set the environment variable PYTHON3_EXECUTABLE to point to it."
        exit 1
    fi
fi

PYTHON_INST_DIR=`$PYTHON3_EXECUTABLE -c "import distutils.sysconfig, sys; sys.stdout.write(distutils.sysconfig.get_python_lib(0,0,''))"`
export PYTHONPATH=$PYYAML_DIR/pyyaml/lib/python3.2/site-packages:$MORSE_DIR/morse/$PYTHON_INST_DIR:/usr/$PYTHON_INST_DIR:/opt/ros/fuerte/lib/python2.7/dist-packages


export MORSE_BLENDER=$BLENDER_DIR/bin/blender

echo "Using PYTHONPATH: $PYTHONPATH"

$MORSE_DIR/morse/bin/morse run $MORSE_DIR/morse/share/morse/examples/scenarii/lorenz_tut_navi.py

