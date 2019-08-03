# pararl
otter project


# test demo 
1. test continuous control mode
To test kinova env, run `python test/keyboard_control.py` and use keyboard to control the end-effector in Cartesian space.

2. test discrete control mode 
run `python test/test_discrete_kinova_control.py' and use keyboard(1,2,3,4,5,6) to control ee to move.


# steps for setting ros enviroment
1. export PATH=~/anaconda3/bin:$PATH
2. source activate py35
3. source ~/ProjectOtter/pararl2/ROS/devel/setup.bash
4. ./pycharm-2018.3.3/bin/pycharm.sh

# steps to set up real robot
open a terminal, then input
1. export PYTHONPATH="/usr/lib/python2.7/dist-packages:$PYTHONPATH"
2. roslaunch kinova_bringup kinova_robot.launch

open another terminal, then input
1. export PYTHONPATH="/usr/lib/python2.7/dist-packages:$PYTHONPATH"
2. rosrun otter_kinova_grasping kinova_control.py 

#steps to set up realsense
1. export PYTHONPATH="/usr/lib/python2.7/dist-packages:$PYTHONPATH"
2. roslaunch otter_kinova_grasping kinova_camera.launch 



### Build your project
1. build ros in Python3.5 terminal in order to use opencv
 ```bash
cd ros/src
catkin_make
source devel/setup.bash
```
2. connect to the kinova 
```
roslaunch kinova_bringup kinova_robot.launch
```

3. open realsense
```angular2
roslaunch otter_kinova_grasping kinova_camera.launch 
```
and you can open `rviz` to visualize the image from the realsense.

4. rosrun kinova_controller
```angular2
export PYTHONPATH="/usr/lib/python2.7/dist-packages:$PYTHONPATH"
rosrun otter_kinova_grasping kinova_control.py 
```

5. run the demo
```angular2
python experiments/myexp/myexp_real_demo.py
```
