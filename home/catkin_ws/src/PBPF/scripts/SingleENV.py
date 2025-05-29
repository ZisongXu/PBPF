#!/usr/bin/python3
#ROS
from concurrent.futures.process import _threads_wakeups
import itertools
import os.path
from pickle import TRUE
from re import T
from ssl import ALERT_DESCRIPTION_ILLEGAL_PARAMETER
from tkinter.tix import Tree
import rospy
import threading
import rospkg
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, TransformStamped, Vector3
from PBPF.msg import object_pose, particle_pose, particle_list, estimated_obj_pose
import tf
import tf.transformations as transformations
from visualization_msgs.msg import Marker
import time
from pyquaternion import Quaternion
import numpy as np
import math
import random
import copy
import os
import signal
import sys
import multiprocessing
import matplotlib.pyplot as plt
import yaml
import pandas as pd
#from sksurgerycore.algorithms.averagequaternions import average_quaternions
from quaternion_averaging import weightedAverageQuaternions
from Particle import Particle
from Object_Pose import Object_Pose
from PhysicsEnv import PhysicsEnv
# - Parmesan
# - Milk
# - SaladDressing
# - Mustard
# - Mayo
# - cracker
# - soup
# - Ketchup

#Class of initialize the simulation model
class SingleENV(multiprocessing.Process):
    def __init__(self, physics_env: PhysicsEnv,
                 pw_T_rob_sim_pose_list_alg, pw_T_obj_obse_obj_list_alg,
                 result_dict, daemon=True):
        super().__init__(daemon=daemon)

        self.env = physics_env
        self.pw_T_rob_sim_pose_list_alg = pw_T_rob_sim_pose_list_alg
        self.pw_T_obj_obse_obj_list_alg = pw_T_obj_obse_obj_list_alg

        self.queue = multiprocessing.Queue()
        self.lock = multiprocessing.Lock()
        self.result = result_dict

    def run(self):
        # This is needed due to how multiprocessing works which will fork the
        # main process, including the seed for numpy random library and as a result, 
        # if the seed is not re-generated in each process, 
        # each process will generate the same noisy trajectory.
        np.random.seed()
        
        self.env.init_env(self.pw_T_rob_sim_pose_list_alg, self.pw_T_obj_obse_obj_list_alg)
        
        while True:
            with self.lock:
                if not self.queue.empty():
                    method, *args = self.queue.get()
                    result = method(self, *args)
                    for key, value in result:
                        self.result[key] = value
            time.sleep(0.00001)
    
    def dummy(self):
        return [('success', True)]

    def get_objects_pose(self, par_index):
        return_results = self.env.get_objects_pose(par_index)
        return return_results

    def isAnyParticleInContact(self):
        return_results = self.env.isAnyParticleInContact()
        return return_results

    def motion_model(self, joint_states, par_index):
        return_results = self.env.motion_model(joint_states, par_index)
        return return_results

    def init_set_sim_robot_JointPosition(self, joint_states):
        self.env.init_set_sim_robot_JointPosition(joint_states)

    def move_robot_JointPosition(self, joint_states):
        return_results = self.env.move_robot_JointPosition(joint_states)
        return return_results

    def compare_distance(self, par_index, pw_T_obj_obse_objects_pose_list, visual_by_DOPE_list, outlier_by_DOPE_list):
        return_results = self.env.compare_distance(par_index, pw_T_obj_obse_objects_pose_list, visual_by_DOPE_list, outlier_by_DOPE_list)
        return return_results

    def set_particle_in_each_sim_env(self, single_particle):
        return_results = self.env.set_particle_in_each_sim_env(single_particle)
        return return_results

    def update_object_pose_PB(self, obj_index, x, y, z, pb_quat, linearVelocity, angularVelocity):
        self.env.update_object_pose_PB(obj_index, x, y, z, pb_quat, linearVelocity, angularVelocity)


    def get_item_pos(self, item_id):
        item_info = self.p_env.getBasePositionAndOrientation(item_id)
        return item_info[0], item_info[1]

    def getLinkStates(self):
        return_results = self.env.getLinkStates()
        return return_results

    
