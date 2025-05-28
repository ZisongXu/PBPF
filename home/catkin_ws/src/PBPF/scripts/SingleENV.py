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

        with open(os.path.expanduser("~/catkin_ws/src/PBPF/config/parameter_info.yaml"), 'r') as file:
            self.parameter_info = yaml.safe_load(file)

        self.RUN_ALG_FLAG = self.parameter_info['run_alg_flag']

        self.PHYSICS_SIMULATION = self.parameter_info['physics_simulation']
        self.update_style_flag = self.parameter_info['update_style_flag']
        self.SIM_TIME_STEP = self.parameter_info['sim_time_step']
        self.PF_UPDATE_FREQUENCY = self.parameter_info['pf_update_frequency']
    
        self.PF_UPDATE_INTERVAL_IN_SIM = self.PF_UPDATE_FREQUENCY[self.RUN_ALG_FLAG] / self.SIM_TIME_STEP

        self.gazebo_flag = self.parameter_info['gazebo_flag']
        self.task_flag = self.parameter_info['task_flag']
        self.SIM_REAL_WORLD_FLAG = self.parameter_info['sim_real_world_flag']
        self.SHOW_RAY = self.parameter_info['show_ray'] 
        self.OBJECT_NAME_LIST = self.parameter_info['object_name_list']
        self.OBJECT_NUM = self.parameter_info['object_num']
        
        self.PANDA_ROBOT_LINK_NUMBER = self.parameter_info['panda_robot_link_number']
        self.MASS_marker = self.parameter_info['MASS_marker']
        self.FRICTION_marker = self.parameter_info['FRICTION_marker']
        
        # ============================================================================

        # Initialization Parameters
        self.sigma_obs_pos_for_init = self.parameter_info['sigma_obs_pos_for_init']
        self.sigma_obs_z_for_init = self.parameter_info['sigma_obs_z_for_init']
        self.sigma_obs_ang_for_init = self.parameter_info['sigma_obs_ang_for_init']
        self.sigma_obs_x_for_init = self.sigma_obs_pos_for_init / math.sqrt(2)
        self.sigma_obs_y_for_init = self.sigma_obs_pos_for_init / math.sqrt(2)

        # ============================================================================

        # Motion Model Parameters
        self.OBJ_MASS_MEAN_DICT = self.parameter_info['obj_mass_mean_dict']
        self.OBJ_MASS_SIGMA_DICT = self.parameter_info['obj_mass_sigma_dict']
        self.OBJ_MASS_MIN_MEAN_DICT = self.parameter_info['obj_mass_min_mean_dict']
        self.OBJ_MASS_NOISE_FLAG = self.parameter_info['obj_mass_noise_flag']
        self.OBJ_FRICTION_MEAN_DICT = self.parameter_info['obj_friction_mean_dict']
        self.OBJ_FRICTION_SIGMA_DICT = self.parameter_info['obj_friction_sigma_dict']
        self.OBJ_FRICTION_MIN_MEAN_DICT = self.parameter_info['obj_friction_min_mean_dict']
        self.OBJ_FRICTION_NOISE_FLAG = self.parameter_info['obj_friction_noise_flag']
        self.OBJ_RESTITUTION_MEAN = self.parameter_info['obj_restitution_mean']
        self.OBJ_RESTITUTION_SIGMA = self.parameter_info['obj_restitution_sigma']
        self.OBJ_MOTION_MODEL_POS_NOISE = self.parameter_info['obj_motion_model_pos_noise']
        self.OBJ_MOTION_MODEL_ANG_NOISE = self.parameter_info['obj_motion_model_ang_noise']
        self.OBJ_MOTION_MODEL_NOISE_FLAG = self.parameter_info['obj_motion_model_noise_flag']

        # # Different labels of m and f are used to test the impact of varying mass and friction on tracking accuracy. 
        # # You can set different parameters as needed. Details are omitted here.
        if self.MASS_marker in {'mA', 'mB', 'mC', 'mD', 'mE', 'mF', 'mG'} and self.FRICTION_marker in {'fA', 'fB', 'fC', 'fD', 'fE', 'fF', 'fG'}:
            self.OBJ_MOTION_MODEL_NOISE_FLAG = False
            self.OBJ_MASS_NOISE_FLAG = False
            self.OBJ_FRICTION_NOISE_FLAG = False
        elif self.MASS_marker in {'mANN', 'mBNN', 'mCNN', 'mDNN', 'mENN', 'mFNN', 'mGNN'} or self.FRICTION_marker in {'fANN', 'fBNN', 'fCNN', 'fDNN', 'fENN', 'fFNN', 'fGNN'}:
            self.OBJ_MOTION_MODEL_NOISE_FLAG = True
            self.OBJ_MASS_NOISE_FLAG = True
            self.OBJ_FRICTION_NOISE_FLAG = True
        else:
            pass

        # ============================================================================

        # Observation Model Parameters
        self.OBJ_SIGMA_POS_FOR_OBS_WEIGHT_DICT = self.parameter_info['obj_sigma_pos_for_obs_weight_dict']
        self.OBJ_SIGMA_ANG_FOR_OBS_WEIGHT_DICT = self.parameter_info['obj_sigma_ang_for_obs_weight_dict']

        # ============================================================================

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

    
