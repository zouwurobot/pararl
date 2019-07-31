#!/usr/bin/python
#import numpy as np
import sys
import os
# os.environ["PYTHONPATH"]="/usr/local/lib/python2.7/dist-packages"
# os.environ["PYTHONPATH"]="/usr/lib/python2.7/dist-package"
# os.environ["ROSPATH"]="/opt/ros/kinetic/share/"
# os.environ["PYTHONPATH"]="/opt/ros/kinetic/lib/python2.7/dist-packages/"
# print(os.environ.keys())
# sys.path.append('/opt/ros/kinetic/share/')
# os.system('. /home/hu/.bashrc')
import sys
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy
rospy.init_node('k')

print('hello world !')