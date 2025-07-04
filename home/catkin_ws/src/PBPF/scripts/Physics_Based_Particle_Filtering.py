#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 10 10:57:49 2021

@author: 12106
"""
from gazebo_msgs.msg import ModelStates
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
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState, CameraInfo, Image, PointCloud2
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, TransformStamped, Vector3
from PBPF.msg import estimated_obj_pose, object_pose, particle_list, particle_pose
import tf
import tf.transformations as transformations
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import message_filters
import cv2
# 
from pyquaternion import Quaternion
from scipy.stats import norm
from scipy.spatial.transform import Rotation as R
import time
import numpy as np
import matplotlib.pyplot as plt
import math
import random
import copy
import os
import signal
import sys
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.pyplot import imsave
import pandas as pd
import multiprocessing
from multiprocessing import Process
import yaml
# import jax.numpy as jnp
# from jax import jit
import heapq
from collections import namedtuple

#from sksurgerycore.algorithms.averagequaternions import average_quaternions
from quaternion_averaging import weightedAverageQuaternions
#class in other files
from Franka_robot import Franka_robot
from Ros_Listener import Ros_Listener
from Particle import Particle
from InitialSimulationModel import InitialSimulationModel
from SingleENV import SingleENV
from Visualisation_World import Visualisation_World
from Create_Scene import Create_Scene
from Object_Pose import Object_Pose
from Robot_Pose import Robot_Pose
from PyBulletEnv import PyBulletEnv

_record_t_begin = time.time()

ROSBAG_TIME = sys.argv[1]
REPEAT_TIME = sys.argv[2]

# parameter_filename = rospy.get_param("~parameter_filename")
with open(os.path.expanduser("~/catkin_ws/src/PBPF/config/parameter_info.yaml"), 'r') as file:
    parameter_info = yaml.safe_load(file)

gazebo_flag = parameter_info['gazebo_flag']
# scene
TASK_FLAG = parameter_info['task_flag'] # parameter_info['task_flag']
# which algorithm to run
run_alg_flag = parameter_info['run_alg_flag'] # PBPF/CVPF
# update mode (pose/time)
UPDATE_STYLE_FLAG = parameter_info['update_style_flag'] # time/pose
SIM_TIME_STEP = parameter_info['sim_time_step'] # time/pose
PF_UPDATE_FREQUENCY = parameter_info['pf_update_frequency'] # time/pose

# observation model
PICK_PARTICLE_RATE = parameter_info['pick_particle_rate']
OPTITRACK_FLAG = parameter_info['optitrack_flag']

VERSION = parameter_info['version'] # old/ray/multiray/
RUNNING_MODEL = parameter_info['running_model'] # PBPF_RGB/PBPF_RGBD/PBPF_D/PBPF_Opti

DEBUG_DEPTH_IMG_FLAG = parameter_info['debug_depth_img_flag'] # old/ray/multiray/depth_img
USE_CONVOLUTION_FLAG = parameter_info['use_convolution_flag'] # old/ray/multiray/depth_img
CONVOLUTION_SIZE = parameter_info['convolution_size'] # old/ray/multiray/depth_img
# the flag is used to determine whether the robot touches the particle in the simulation
simRobot_touch_par_flag = 0
OBJECT_NUM = parameter_info['object_num']
ROBOT_NUM = parameter_info['robot_num']
OBJ_SHAPE = parameter_info['obj_shape']
#####################################
OBJ_VISIBLESCORE_OBSE_WORK_THRESHOLD = parameter_info['obj_VisibleScore_obse_work_threshold']
OBJ_VISIBLESCORE_OBSE_FAIL_THRESHOLD_HIGH = parameter_info['obj_VisibleScore_obse_fail_threshold_high']
OBJ_VISIBLESCORE_OBSE_FAIL_THRESHOLD_LOW = parameter_info['obj_VisibleScore_obse_fail_threshold_low']
OBJ_VISIBLESCORE_OBSE_FAIL_WEIGHT_HIGH = parameter_info['obj_VisibleScore_obse_fail_weight_high']
OBJ_VISIBLESCORE_OBSE_FAIL_WEIGHT_LOW = parameter_info['obj_VisibleScore_obse_fail_weight_low']
OBJ_OUTLIER_STANDARD = parameter_info['obj_outlier_standard']
#####################################


SIM_REAL_WORLD_FLAG = parameter_info['sim_real_world_flag']

LOCATE_CAMERA_FLAG = parameter_info['locate_camera_flag']

PARTICLE_NUM = parameter_info['particle_num']

OBJECT_NAME_LIST = parameter_info['object_name_list']

CAMERA_INFO_TOPIC_COLOR = parameter_info['camera_info_topic_color'] # /camera/color/camera_info
CAMERA_INFO_TOPIC_DEPTH = parameter_info['camera_info_topic_depth'] # /camera/depth/camera_info

NEARVAL = parameter_info['nearVal'] # 57.86
FARVAL = parameter_info['farVal'] # 57.86

DEPTH_IMAGE_CUT_FLAG = parameter_info['depth_image_cut_flag'] 
PERSP_TO_ORTHO_FLAG = parameter_info['persp_to_ortho_flag'] 
ORTHO_TO_PERSP_FLAG = parameter_info['ortho_to_persp_flag'] 
DEPTH_DIFF_VALUE_0_1_FLAG = parameter_info['depth_diff_value_0_1_flag'] 
DEPTH_DIFF_VALUE_0_1_THRESHOLD = parameter_info['depth_diff_value_0_1_threshold'] 
DEPTH_DIFF_VALUE_0_1_ALPHA = parameter_info['depth_diff_value_0_1_alpha'] 
DEPTH_MASK_FLAG = parameter_info['depth_mask_flag'] 
DEPTH_MASK_VK_FLAG = parameter_info['depth_mask_vk_flag'] 
COMBINE_PARTICLE_DEPTH_MASK_FLAG = parameter_info['combine_particle_depth_mask_flag'] 
SHOW_PARTICLE_DEPTH_IMAGE_TO_POINT_CLOUD_FLAG = parameter_info['show_particle_depth_image_to_point_cloud_flag'] 
IGNORE_EDGE_PIXELS = parameter_info['ignore_edge_pixels'] 

COMPARE_DEPTH_IMG_VK = parameter_info['compare_depth_img_vk']
VISIBILITY_COMPUTE_SEPARATE_FLAG = parameter_info['visibility_compute_separate_flag']
VISIBILITY_COMPUTE_VK = parameter_info['visibility_compute_vk']
PROCESS_MODEL_FLAG = parameter_info['process_model_flag'] # thread/multiprocess/normal

RECORD_RESULTS_FLAG = parameter_info['record_results_flag'] 
RECORD_TIME_CONSUMPTION_FLAG = parameter_info['record_time_consumption_flag'] 
PRINT_FLAG = parameter_info['print_flag']

PRINT_SCORE_FLAG = parameter_info['print_score_flag'] 
SHOW_RAY = parameter_info['show_ray']
RENDER_FLAG = parameter_info['render_flag'] # 'vk', 'pb'
PANDA_ROBOT_LINK_NUMBER = parameter_info['panda_robot_link_number']
DRAW_WIREFRAME_FLAG = parameter_info['draw_wireframe_flag']
TEST_INITIAL_RENDERED_DEPTH_IMAGE_FLAG = parameter_info['test_initial_rendered_depth_image_flag']

MASS_marker = parameter_info['MASS_marker']
FRICTION_marker = parameter_info['FRICTION_marker']

PHYSICS_SIMULATION = parameter_info['physics_simulation']

if RENDER_FLAG == 'vk':
    print("I am using Vulkan to generate Depth Image")
elif RENDER_FLAG == 'pb': 
    print("I am using Pybullet to generate Depth Image")

# ==============================================================================================================================

# create parameters for recording data
_record_PBPF_esti_pose_list = []
_record_PBPF_par_pose_list = []
_record_obse_pose_list = []
_record_GT_pose_list = []
_record_obse_pose_first_flag = 0
_record_time_list = []
_record_GT_time_list = []

_boss_obse_err_ADD_df_list = [0]*OBJECT_NUM
_obse_panda_step = 0
_boss_PBPF_err_ADD_df_list = [0]*OBJECT_NUM
_PBPF_panda_step = 0
_boss_GT_err_ADD_df_list = [0]*OBJECT_NUM
_GT_panda_step = 0
# _boss_GT_visibility_ADD_df_list = [0]*OBJECT_NUM
_GTV_panda_step = 0
_boss_par_err_ADD_df_list = [0]*PARTICLE_NUM
_par_panda_step = 0

for obj_index in range(OBJECT_NUM):
    _boss_obse_err_ADD_df = pd.DataFrame(columns=['step','time','pos_x','pos_y','pos_z','ori_x','ori_y','ori_z','ori_w','alg','obj','scene','particle_num','ray_type','obj_name','mass','friction'],index=[])
    _boss_PBPF_err_ADD_df = pd.DataFrame(columns=['step','time','pos_x','pos_y','pos_z','ori_x','ori_y','ori_z','ori_w','alg','obj','scene','particle_num','ray_type','obj_name','mass','friction'],index=[])
    _boss_GT_err_ADD_df = pd.DataFrame(columns=['step','time','pos_x','pos_y','pos_z','ori_x','ori_y','ori_z','ori_w','alg','obj','scene','particle_num','ray_type','obj_name','mass','friction'],index=[])
    _boss_obse_err_ADD_df_list[obj_index] = _boss_obse_err_ADD_df
    _boss_PBPF_err_ADD_df_list[obj_index] = _boss_PBPF_err_ADD_df
    _boss_GT_err_ADD_df_list[obj_index] = _boss_GT_err_ADD_df
    # _boss_GT_visibility_ADD_df = pd.DataFrame(columns=['step','time','pos_x','pos_y','pos_z','ori_x','ori_y','ori_z','ori_w','alg','obj','scene','particle_num','visibilityScore','obj_name','mass','friction'],index=[])
    # _boss_GT_visibility_ADD_df = pd.DataFrame(columns=['step','time','visibilityScore'],index=[])
    # _boss_GT_visibility_ADD_df_list[obj_index] = _boss_GT_visibility_ADD_df
for par_index in range(PARTICLE_NUM):
    _boss_par_err_ADD_df = pd.DataFrame(columns=['step','time','pos_x','pos_y','pos_z','ori_x','ori_y','ori_z','ori_w','alg','obj','scene','particle_num','ray_type','obj_name','mass','friction'],index=[])
    _boss_par_err_ADD_df_list[par_index] = _boss_par_err_ADD_df

# ==============================================================================================================================

# some debug parameters
Motion_model_time_consuming_list = []
Render_depth_image_time_comsuming_list = []
Compare_depth_image_time_consuming_list = []
Compare_distance_time_consuming_list = []
Compute_visibility_score_time_consuming_list = []
Observation_model_time_consuming_list = []
Resample_time_consuming_list = []
Deepcopy_time_consuming_list = []
Setpub_time_consuming_list = []
One_update_time_cosuming_list = []

Motion_model_time_consumption = 0
Render_depth_image_time_consumption = 0
Compare_depth_image_time_consumption = 0
Compare_distance_time_consumption = 0
Compute_visibility_score_time_consumption = 0
Observation_model_time_consumption = 0
Resample_time_consumption = 0
Deepcopy_time_consumption = 0
Setpub_time_consumption = 0
One_update_time_consumption = 0

_boss_time_consumption_df = pd.DataFrame(columns=['step','time',
                                                  'motion_model',
                                                  'render_depth_img','compare_depth_img',
                                                  'compare_dis','compute_visibility_score',
                                                  'resample','deepcopy','setpub','obs_model',
                                                  'one_update_time'],index=[])
_time_record_index = 0

# ==============================================================================================================================

# vulkan
from pathlib import Path
import sys
sys.path.insert( 1, str(Path( __file__ ).parent.parent.absolute() / "bin") )
## Import module
import vkdepth
print("Launch Vkdepth successfully")

# ==============================================================================================================================

def compute_pos_err_bt_2_points(pos1, pos2):
    """
    compute the position difference between two objects
    """
    x1=pos1[0]
    y1=pos1[1]
    z1=pos1[2]
    x2=pos2[0]
    y2=pos2[1]
    z2=pos2[2]
    x_d = x1-x2
    y_d = y1-y2
    z_d = z1-z2
    distance = math.sqrt(x_d ** 2 + y_d ** 2 + z_d ** 2)
    return distance

def compute_ang_err_bt_2_points(object1_ori, object2_ori):
    """
    compute the rotation difference between two objects
    """
    #[x, y, z, w]
    obj1_ori = copy.deepcopy(object1_ori)
    obj2_ori = copy.deepcopy(object2_ori)
    obj1_ori_quat = quaternion_correction(obj1_ori) # x,y,z,w
    obj2_ori_quat = quaternion_correction(obj2_ori) # x,y,z,w

    #[w, x, y, z]
    obj1_quat = Quaternion(x = obj1_ori_quat[0], y = obj1_ori_quat[1], z = obj1_ori_quat[2], w = obj1_ori_quat[3]) # Quaternion(): w,x,y,z
    obj2_quat = Quaternion(x = obj2_ori_quat[0], y = obj2_ori_quat[1], z = obj2_ori_quat[2], w = obj2_ori_quat[3]) # Quaternion(): w,x,y,z
    diff_bt_o1_o2 = obj2_quat * obj1_quat.inverse
    cos_theta_over_2 = diff_bt_o1_o2.w
    sin_theta_over_2 = math.sqrt(diff_bt_o1_o2.x ** 2 + diff_bt_o1_o2.y ** 2 + diff_bt_o1_o2.z ** 2)
    theta_over_2 = math.atan2(sin_theta_over_2, cos_theta_over_2)
    theta = theta_over_2 * 2
    theta = abs(theta)
    return theta

def find_closest_pose_metrics(obj_index, particle_cloud_pub, pw_T_obj_obse_pose_new):
    """
    Computes the minimum translational and rotational differences between a reference 6D pose and a list of candidate 6D poses.
    """
    par_cloud_for_compute = particle_cloud_pub
    obj_obse_pos_new = pw_T_obj_obse_pose_new[0]
    obj_obse_ori_new = pw_T_obj_obse_pose_new[1]
    par_dis_list = []
    par_ang_list = []
    par_cloud_length = len(par_cloud_for_compute)
    for par_index in range(par_cloud_length):
        par_pos = par_cloud_for_compute[par_index][obj_index].pos
        par_ori = par_cloud_for_compute[par_index][obj_index].ori

        dis_obseCur_parOld = compute_pos_err_bt_2_points(obj_obse_pos_new, par_pos)
        ang_obseCur_parOld = compute_ang_err_bt_2_points(obj_obse_ori_new, par_ori)
        par_dis_list.append(dis_obseCur_parOld)
        par_ang_list.append(ang_obseCur_parOld)

    def find_smallest_value(number_list):
        num = int(PARTICLE_NUM / 20.0)
        num = 1
        if num == 0 :
            num = num + 1
        return heapq.nsmallest(num, number_list)

    new_dis_list = find_smallest_value(par_dis_list)
    new_ang_list = find_smallest_value(par_ang_list)

    minDis_obseCur_parOld = new_dis_list[-1]
    minAng_obseCur_parOld = new_ang_list[-1]

    return minDis_obseCur_parOld, minAng_obseCur_parOld

def compute_transformation_matrix(a_pos, a_ori, b_pos, b_ori):
    """
    Computes the 4×4 homogeneous transformation matrix representing the pose of frame B
    relative to frame A, given the poses of A and B in the world frame.
    """
    # ow_T_a_3_3 = transformations.quaternion_matrix(a_ori)
    # ow_T_a_4_4 = rotation_4_4_to_transformation_4_4(ow_T_a_3_3,a_pos)
    # ow_T_b_3_3 = transformations.quaternion_matrix(b_ori)
    # ow_T_b_4_4 = rotation_4_4_to_transformation_4_4(ow_T_b_3_3,b_pos)
    # a_T_ow_4_4 = np.linalg.inv(ow_T_a_4_4)
    # a_T_b_4_4 = np.dot(a_T_ow_4_4,ow_T_b_4_4)


    ow_T_a_pos = [a_pos[0], a_pos[1], a_pos[2]]
    ow_T_a_ori = [a_ori[0], a_ori[1], a_ori[2], a_ori[3]] # x,y,z,w
    ow_T_a_ori_anti = [a_ori[3], a_ori[0], a_ori[1], a_ori[2]] # w,x,y,z
    tmp_q = Quaternion(ow_T_a_ori_anti)
    ow_T_a_3_3 = tmp_q.rotation_matrix
    ow_T_a_3_4 = np.c_[ow_T_a_3_3, ow_T_a_pos]  # Add position to create 3x4 matrix
    ow_T_a_4_4 = np.r_[ow_T_a_3_4, [[0, 0, 0, 1]]]  # Convert to 4x4 homogeneous matrix

    ow_T_b_pos = [b_pos[0], b_pos[1], b_pos[2]]
    ow_T_b_ori = [b_ori[0], b_ori[1], b_ori[2], b_ori[3]] # x,y,z,w
    ow_T_b_ori_anti = [b_ori[3], b_ori[0], b_ori[1], b_ori[2]] # w,x,y,z
    tmp_q = Quaternion(ow_T_b_ori_anti)
    ow_T_b_3_3 = tmp_q.rotation_matrix
    ow_T_b_3_4 = np.c_[ow_T_b_3_3, ow_T_b_pos]  # Add position to create 3x4 matrix
    ow_T_b_4_4 = np.r_[ow_T_b_3_4, [[0, 0, 0, 1]]]  # Convert to 4x4 homogeneous matrix


    # ow_T_a_3_3 = np.array(p.getMatrixFromQuaternion(a_ori)).reshape(3, 3)
    # ow_T_a_3_4 = np.c_[ow_T_a_3_3, a_pos]  # Add position to create 3x4 matrix
    # ow_T_a_4_4 = np.r_[ow_T_a_3_4, [[0, 0, 0, 1]]]  # Convert to 4x4 homogeneous matrix

    # ow_T_b_3_3 = np.array(p.getMatrixFromQuaternion(b_ori)).reshape(3, 3)
    # ow_T_b_3_4 = np.c_[ow_T_b_3_3, b_pos]  # Add position to create 3x4 matrix
    # ow_T_b_4_4 = np.r_[ow_T_b_3_4, [[0, 0, 0, 1]]]  # Convert to 4x4 homogeneous matrix

    a_T_ow_4_4 = np.linalg.inv(ow_T_a_4_4)
    a_T_b_4_4 = np.dot(a_T_ow_4_4,ow_T_b_4_4)
    return a_T_b_4_4

def get_item_pos(pybullet_env, item_id):
    """
    get pose of item in given PyBullet Env
    """
    item_info = pybullet_env.getBasePositionAndOrientation(item_id)
    return item_info[0],item_info[1]

def take_easy_gaussian_value(mean, sigma):
    """
    random values generated from a Gaussian distribution
    """
    normal = random.normalvariate(mean, sigma)
    return normal

def angle_correction(angle):
    """
    make sure all angles all between -pi and +pi
    """
    if math.pi <= angle <= (3.0 * math.pi):
        angle = angle - 2 * math.pi
    elif -(3.0 * math.pi) <= angle <= -math.pi:
        angle = angle + 2 * math.pi
    angle = abs(angle)
    return angle
 
def quaternion_correction(quaternion): # x,y,z,w
    """
    make sure all quaternions all between -pi and +pi
    """
    new_quat = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]) # w,x,y,z
    cos_theta_over_2 = new_quat.w
    sin_theta_over_2 = math.sqrt(new_quat.x ** 2 + new_quat.y ** 2 + new_quat.z ** 2)
    theta_over_2 = math.atan2(sin_theta_over_2,cos_theta_over_2)
    theta = theta_over_2 * 2.0
    while theta >= math.pi:
        theta = theta - 2.0*math.pi
    while theta <= -math.pi:
        theta = theta + 2.0*math.pi
    new_quaternion = [math.sin(theta/2.0)*(new_quat.x/sin_theta_over_2), 
                      math.sin(theta/2.0)*(new_quat.y/sin_theta_over_2), 
                      math.sin(theta/2.0)*(new_quat.z/sin_theta_over_2), 
                      math.cos(theta/2.0)]
    return new_quaternion # x,y,z,w

def _get_position_from_matrix44(a_T_b_4_4):
    """
    get position from transformation matrix
    """
    x = a_T_b_4_4[0][3]
    y = a_T_b_4_4[1][3]
    z = a_T_b_4_4[2][3]
    position = [x, y, z]
    return position

def _get_quaternion_from_matrix(a_T_b_4_4):
    """
    get quaternion matrix (4×4) from transformation matrix
    """
    rot_matrix = a_T_b_4_4[:3, :3]
    rotation = R.from_matrix(rot_matrix)
    quaternion = rotation.as_quat()
    return quaternion

# def publish_ray_trace_info(particle_cloud_pub):
#    par_pose_list = list(range(PARTICLE_NUM))
#    for par_index in range(PARTICLE_NUM):
#        par_pose = particle_pose()
#        obj_pose_list = []
#        for obj_index in range(OBJECT_NUM):
#            obj_pose = object_pose()
#            obj_info = particle_cloud_pub[par_index][obj_index]
#            obj_pose.name = obj_info.par_name
#            obj_pose.pose.position.x = obj_info.pos[0]
#            obj_pose.pose.position.y = obj_info.pos[1]
#            obj_pose.pose.position.z = obj_info.pos[2]
#            obj_pose_list.append(obj_pose)
#        par_pose.objects = obj_pose_list
#        par_pose_list[par_index] = par_pose
#    par_list.particles = par_pose_list
#    pub_ray_trace.publish(par_list)

def _publish_par_pose_info(particle_cloud_pub):
    global _par_panda_step
    par_pose_list = list(range(PARTICLE_NUM))
    rob_par_pose_list = list(range(PARTICLE_NUM))
    for par_index in range(PARTICLE_NUM):
        par_pose = particle_pose()
        par_pose.particles = par_index
        rob_par_pose = particle_pose()
        rob_par_pose.particles = par_index
        obj_pose_list = []
        rob_obj_pose_list = []

        for obj_index in range(OBJECT_NUM):
            obj_info = particle_cloud_pub[par_index][obj_index]
            if RECORD_RESULTS_FLAG == True:
                obj_name = obj_info.par_name
                scene = "scene"+str(TASK_FLAG)
                obj_scene = obj_info.par_name+"_scene"+str(TASK_FLAG)
                _record_t = time.time()
                # x, y, z ,w
                _boss_par_err_ADD_df_list[par_index].loc[_par_panda_step] = [_par_panda_step, _record_t - _record_t_begin, obj_info.pos[0], obj_info.pos[1], obj_info.pos[2], obj_info.ori[0], obj_info.ori[1], obj_info.ori[2], obj_info.ori[3], RUNNING_MODEL, obj_name, scene, PARTICLE_NUM, VERSION, obj_name, MASS_marker, FRICTION_marker]
                _par_panda_step = _par_panda_step + 1
    
            pw_T_par_pos = [obj_info.pos[0], obj_info.pos[1], obj_info.pos[2]]
            pw_T_par_ori = [obj_info.ori[0], obj_info.ori[1], obj_info.ori[2], obj_info.ori[3]] # x,y,z,w
            pw_T_par_ori_anti = [obj_info.ori[3], obj_info.ori[0], obj_info.ori[1], obj_info.ori[2]] # w,x,y,z
            tmp_q = Quaternion(pw_T_par_ori_anti)
            pw_T_par_3_3 = tmp_q.rotation_matrix
            pw_T_par_3_4 = np.c_[pw_T_par_3_3, pw_T_par_pos]  # Add position to create 3x4 matrix
            pw_T_par_4_4 = np.r_[pw_T_par_3_4, [[0, 0, 0, 1]]]  # Convert to 4x4 homogeneous matrix
            rob_T_pw_4_4 = np.linalg.inv(_pw_T_rob_sim_4_4)
            rob_T_par_4_4 = rob_T_pw_4_4 @ pw_T_par_4_4

            rob_T_par_pos = [rob_T_par_4_4[0][3], rob_T_par_4_4[1][3], rob_T_par_4_4[2][3]]
            rob_T_par_ori = transformations.quaternion_from_matrix(rob_T_par_4_4)

            obj_pose = object_pose()
            obj_pose.id = obj_index
            obj_pose.name = obj_info.par_name
            obj_pose.pose.position.x = obj_info.pos[0]
            obj_pose.pose.position.y = obj_info.pos[1]
            obj_pose.pose.position.z = obj_info.pos[2]
            obj_pose.pose.orientation.x = obj_info.ori[0]
            obj_pose.pose.orientation.y = obj_info.ori[1]
            obj_pose.pose.orientation.z = obj_info.ori[2]
            obj_pose.pose.orientation.w = obj_info.ori[3]
            obj_pose_list.append(obj_pose)

            rob_obj_pose = object_pose()
            rob_obj_pose.id = obj_index
            rob_obj_pose.name = obj_info.par_name
            rob_obj_pose.pose.position.x = rob_T_par_pos[0]
            rob_obj_pose.pose.position.y = rob_T_par_pos[1]
            rob_obj_pose.pose.position.z = rob_T_par_pos[2]
            rob_obj_pose.pose.orientation.x = rob_T_par_ori[0]
            rob_obj_pose.pose.orientation.y = rob_T_par_ori[1]
            rob_obj_pose.pose.orientation.z = rob_T_par_ori[2]
            rob_obj_pose.pose.orientation.w = rob_T_par_ori[3]
            rob_obj_pose_list.append(rob_obj_pose)

        par_pose.objects = obj_pose_list
        par_pose_list[par_index] = par_pose

        rob_par_pose.objects = rob_obj_pose_list
        rob_par_pose_list[par_index] = rob_par_pose
        
    par_list.particles = par_pose_list
    pub_par_pose.publish(par_list)
    rob_par_list.particles = rob_par_pose_list
    rob_pub_par_pose.publish(rob_par_list)
            
def _publish_esti_pose_info(estimated_object_set):
    global _PBPF_panda_step
    # global _boss_PBPF_err_ADD_df_list
    esti_pose_list = []
    for obj_index in range(OBJECT_NUM):
        esti_pose = object_pose()
        esti_obj_info = estimated_object_set[obj_index]
        esti_pose.name = esti_obj_info.obj_name
        esti_pose.pose.position.x = esti_obj_info.pos[0]
        esti_pose.pose.position.y = esti_obj_info.pos[1]
        esti_pose.pose.position.z = esti_obj_info.pos[2]
        esti_pose.pose.orientation.x = esti_obj_info.ori[0]
        esti_pose.pose.orientation.y = esti_obj_info.ori[1]
        esti_pose.pose.orientation.z = esti_obj_info.ori[2]
        esti_pose.pose.orientation.w = esti_obj_info.ori[3]
        esti_pose_list.append(esti_pose)

        if RECORD_RESULTS_FLAG == True:
            obj = esti_obj_info.obj_name
            scene = "scene"+str(TASK_FLAG)
            obj_scene = esti_obj_info.obj_name+"_scene"+str(TASK_FLAG)
            obj_name = esti_obj_info.obj_name
            _record_t = time.time()
            # x, y, z ,w
            _boss_PBPF_err_ADD_df_list[obj_index].loc[_PBPF_panda_step] = [_PBPF_panda_step, _record_t - _record_t_begin, esti_obj_info.pos[0], esti_obj_info.pos[1], esti_obj_info.pos[2], esti_obj_info.ori[0], esti_obj_info.ori[1], esti_obj_info.ori[2], esti_obj_info.ori[3], RUNNING_MODEL, obj, scene, PARTICLE_NUM, VERSION, obj_name, MASS_marker, FRICTION_marker]                                    

    _PBPF_panda_step = _PBPF_panda_step + 1

    esti_obj_list.objects = esti_pose_list 
    pub_esti_pose.publish(esti_obj_list)

    if RECORD_RESULTS_FLAG == True:
        _record_PBPF_esti_pose_list.append(estimated_object_set)
            
    for obj_index in range(OBJECT_NUM):
        pose_PBPF = PoseStamped()
        pose_PBPF.pose.position.x = estimated_object_set[obj_index].pos[0]
        pose_PBPF.pose.position.y = estimated_object_set[obj_index].pos[1]
        pose_PBPF.pose.position.z = estimated_object_set[obj_index].pos[2]
        pose_PBPF.pose.orientation.x = estimated_object_set[obj_index].ori[0]
        pose_PBPF.pose.orientation.y = estimated_object_set[obj_index].ori[1]
        pose_PBPF.pose.orientation.z = estimated_object_set[obj_index].ori[2]
        pose_PBPF.pose.orientation.w = estimated_object_set[obj_index].ori[3]
        pub_PBPF_list[obj_index].publish(pose_PBPF)

def process_esti_pose_from_rostopic(estimated_object_set):
    esti_pose_list = []
    for obj_index in range(OBJECT_NUM):
        esti_obj_info = estimated_object_set[obj_index]
        esti_obj_pos_x = esti_obj_info.pos[0]
        esti_obj_pos_y = esti_obj_info.pos[1]
        esti_obj_pos_z = esti_obj_info.pos[2]
        esti_obj_ori_x = esti_obj_info.ori[0]
        esti_obj_ori_y = esti_obj_info.ori[1]
        esti_obj_ori_z = esti_obj_info.ori[2]
        esti_obj_ori_w = esti_obj_info.ori[3]
        esti_pose = [[esti_obj_pos_x, esti_obj_pos_y, esti_obj_pos_z], [esti_obj_ori_x, esti_obj_ori_y, esti_obj_ori_z, esti_obj_ori_w]] # x, y, z, w
        esti_pose_list.append(esti_pose)
    return esti_pose_list

def _get_camera_intrinsic_params(camera_info_topic_name):
    """
    get camera intrinsic params
    """
    camera_info = None
    def camera_info_callback(data):
        nonlocal camera_info
        camera_info = data
    rospy.Subscriber(camera_info_topic_name, CameraInfo, camera_info_callback)
    while camera_info is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    fx = camera_info.K[0]  # Focal length in x-axis
    fy = camera_info.K[4]  # Focal length in y-axis
    cx = camera_info.K[2]  # Principal point in x-axis
    cy = camera_info.K[5]  # Principal point in y-axis

    image_height = camera_info.height
    image_width = camera_info.width

    Camera = namedtuple("Camera", "fx fy cx cy image_height image_width")
    camera_intrinsic_parameters = Camera(fx, fy, cx, cy, image_height, image_width)
    return camera_intrinsic_parameters 

def _compute_estimate_pos_of_object(particle_cloud):
    esti_objs_cloud = []
    # remenber after resampling weight of each particle is the same
    for obj_index in range(OBJECT_NUM):
        x_set = 0
        y_set = 0
        z_set = 0
        w_set = 0
        quaternions = []
        qws = []
        for index, particle in enumerate(particle_cloud):
            x_set = x_set + particle[obj_index].pos[0] * particle[obj_index].w
            y_set = y_set + particle[obj_index].pos[1] * particle[obj_index].w
            z_set = z_set + particle[obj_index].pos[2] * particle[obj_index].w
            q = quaternion_correction(particle[obj_index].ori)
            qws.append(particle[obj_index].w)
            quaternions.append([q[0], q[1], q[2], q[3]])
            w_set = w_set + particle[obj_index].w
        q = weightedAverageQuaternions(np.array(quaternions), np.array(qws))
        ###################################
        esti_obj_pos_x = x_set/w_set
        esti_obj_pos_y = y_set/w_set
        esti_obj_pos_z = z_set/w_set
        esti_obj_pos = [esti_obj_pos_x, esti_obj_pos_y, esti_obj_pos_z]
        esti_obj_ori_x = q[0]
        esti_obj_ori_y = q[1]
        esti_obj_ori_z = q[2]
        esti_obj_ori_w = q[3]
        esti_obj_ori = [esti_obj_ori_x, esti_obj_ori_y, esti_obj_ori_z, esti_obj_ori_w]
        ###################################
        est_obj_pose = Object_Pose(OBJECT_NAME_LIST[obj_index], obj_index, esti_obj_pos, esti_obj_ori, obj_index)
        esti_objs_cloud.append(est_obj_pose)
    return esti_objs_cloud

def _vk_config_setting():
    depth_img_height = RESOLUTION_DEPTH[0] # 480
    depth_img_width = RESOLUTION_DEPTH[1] # 848
    ## Create context
    vk_config = vkdepth.ContextConfig()
    vk_config.render_size(depth_img_width, depth_img_height) # width(848), height(480)
    return vk_config

def _vk_camera_setting(pw_T_camD_tf_4_4, camD_T_camVk_4_4):
    ## Setup vk_camera
    vk_camera = vkdepth.Camera()

    pw_T_camD_tf_4_4_ = copy.deepcopy(pw_T_camD_tf_4_4)
    camD_T_camVk_4_4_ = copy.deepcopy(camD_T_camVk_4_4)
    pw_T_camVk_4_4_ = np.dot(pw_T_camD_tf_4_4_, camD_T_camVk_4_4_)
    
    trick_matrix = np.array([[ 1, 0, 0,-0.010],
                             [ 0, 1, 0,-0.010],
                             [ 0, 0, 1,-0.00],
                             [ 0, 0, 0, 1]])
    pw_T_camVk_4_4_ = np.dot(pw_T_camVk_4_4_, trick_matrix)

    pw_T_camVk_pos = _get_position_from_matrix44(pw_T_camVk_4_4_)
    x_pos = pw_T_camVk_pos[0]
    y_pos = pw_T_camVk_pos[1]
    z_pos = pw_T_camVk_pos[2]
    pw_T_camVk_ori = _get_quaternion_from_matrix(pw_T_camVk_4_4_) # x, y, z, w
    x_ori = pw_T_camVk_ori[0]
    y_ori = pw_T_camVk_ori[1]
    z_ori = pw_T_camVk_ori[2]
    w_ori = pw_T_camVk_ori[3]

    vk_camera.set_near_far(NEARVAL, FARVAL)
    vk_camera.set_aspect_wh(WIDTH_DEPTH, HEIGHT_DEPTH) # width: 848, height: 480
    vk_camera.set_position(x_pos, y_pos, z_pos) # x, y, z
    vk_camera.set_orientation_quat(w_ori, x_ori, y_ori, z_ori) # w, x, y, z

    return vk_camera, pw_T_camVk_4_4_

def _vk_load_target_objects_meshes():
    global _vk_context
    vk_obj_id_list_ = [0] * OBJECT_NUM
    for obj_index in range(OBJECT_NUM):
        obj_name = OBJECT_NAME_LIST[obj_index] # "cracker"/"soup"/"Ketchup"
        obj_id = _vk_context.load_model("assets/meshes/"+obj_name+".vkdepthmesh") 
        vk_obj_id_list_[obj_index] = obj_id
    return vk_obj_id_list_

def _vk_load_robot_meshes():
    # There are actually 13 links, of which "link8" and "panda_grasptarget" have no entities.
    ## "link0,1,2,3,4,5,6,7", "panda_hand", "panda_left_finger", "panda_right_finger"
    # index:0,1,2,3,4,5,6,7,   9,            10,                  11    
    global _vk_context
    vk_rob_link_id_list_ = [0] * PANDA_ROBOT_LINK_NUMBER # 11
    for link_index in range(PANDA_ROBOT_LINK_NUMBER):
        if link_index < 8:
            rob_link_id = _vk_context.load_model("assets/meshes/link"+str(link_index)+".vkdepthmesh")
        elif link_index == 8:
            rob_link_id = _vk_context.load_model("assets/meshes/hand.vkdepthmesh")
        elif link_index == 9:
            rob_link_id = _vk_context.load_model("assets/meshes/left_finger.vkdepthmesh")
        elif link_index == 10:
            rob_link_id = _vk_context.load_model("assets/meshes/right_finger.vkdepthmesh")
        vk_rob_link_id_list_[link_index] = rob_link_id
    return vk_rob_link_id_list_


def _vk_load_env_objects_meshes():
    """
    Build Experimental Environment
    """
    global _vk_context
    vk_other_id_list_ = []
    vk_other_obj_info_list_ = []

    # table
    other_obj_id = _vk_context.load_model("assets/meshes/table.vkdepthmesh")
    vk_other_id_list_.append(other_obj_id)
    rot = R.from_euler('xyz', [0,0,0])
    table_ori_array = rot.as_quat()
    table_ori_tuple = tuple(table_ori_array)
    vk_other_obj_info = Object_Pose(obj_name='table', obj_id=0, pos=[0.46, -0.01, 0.702], ori=table_ori_tuple, index=0) # ori: x, y, z, w   
    vk_other_obj_info_list_.append(vk_other_obj_info)
    # board
    other_obj_id = _vk_context.load_model("assets/meshes/board.vkdepthmesh")
    vk_other_id_list_.append(other_obj_id)
    rot = R.from_euler('xyz', [math.pi/2,math.pi/2,0])
    board_ori_array = rot.as_quat()
    board_ori_tuple = tuple(board_ori_array)
    vk_other_obj_info = Object_Pose(obj_name='board', obj_id=1, pos=[0.274, 0.581, 0.87575], ori=board_ori_tuple, index=0) # ori: x, y, z, w           
    vk_other_obj_info_list_.append(vk_other_obj_info)
    # barrier 1,2,3
    # -----------------------------------------------------------------------
    other_obj_id = _vk_context.load_model("assets/meshes/barrier.vkdepthmesh")
    vk_other_id_list_.append(other_obj_id)
    rot = R.from_euler('xyz', [0,math.pi/2,0])
    barrier1_ori_array = rot.as_quat()
    barrier1_ori_tuple = tuple(barrier1_ori_array)
    vk_other_obj_info = Object_Pose(obj_name='barrier1', obj_id=0, pos=[-0.694, 0.443, 0.895], ori=barrier1_ori_tuple, index=0) # ori: x, y, z, w          
    vk_other_obj_info_list_.append(vk_other_obj_info)
    # -----------------------------------------------------------------------
    other_obj_id = _vk_context.load_model("assets/meshes/barrier.vkdepthmesh")
    vk_other_id_list_.append(other_obj_id)
    rot = R.from_euler('xyz', [0,math.pi/2,0])
    barrier2_ori_array = rot.as_quat()
    barrier2_ori_tuple = tuple(barrier2_ori_array)
    vk_other_obj_info = Object_Pose(obj_name='barrier2', obj_id=0, pos=[-0.694, -0.607, 0.895], ori=barrier2_ori_tuple, index=0) # ori: x, y, z, w          
    vk_other_obj_info_list_.append(vk_other_obj_info)
    # -----------------------------------------------------------------------
    other_obj_id = _vk_context.load_model("assets/meshes/barrier.vkdepthmesh")
    vk_other_id_list_.append(other_obj_id)
    rot = R.from_euler('xyz', [0,math.pi/2,math.pi/2])
    barrier3_ori_array = rot.as_quat()
    barrier3_ori_tuple = tuple(barrier3_ori_array)
    vk_other_obj_info = Object_Pose(obj_name='barrier3', obj_id=0, pos=[0.459, -0.972, 0.895], ori=barrier3_ori_tuple, index=0) # ori: x, y, z, w          
    vk_other_obj_info_list_.append(vk_other_obj_info) 
    # pringles
    if TASK_FLAG == '1':
        other_obj_id = _vk_context.load_model("assets/meshes/pringles.vkdepthmesh")
        vk_other_id_list_.append(other_obj_id)
        vk_other_obj_info = Object_Pose(obj_name='pringles', obj_id=0, pos=[0.6652218209791124, 0.058946644391304814, 0.8277292172960276], ori=[ 0.67280124, -0.20574896, -0.20600051, 0.68012472], index=0) # ori: x, y, z, w           
        vk_other_obj_info_list_.append(vk_other_obj_info) 

    return vk_other_id_list_, vk_other_obj_info_list_

def _vk_load_meshes():
    global _vk_context
    vk_obj_id_list = [0] * OBJECT_NUM
    vk_rob_link_id_list = [0] * PANDA_ROBOT_LINK_NUMBER # 11
    vk_other_id_list = []
    vk_other_obj_info_list = []
    # load tracking objects meshes
    vk_obj_id_list = _vk_load_target_objects_meshes()
    # load robot meshes
    vk_rob_link_id_list = _vk_load_robot_meshes()
    # load env objects meshes
    vk_other_id_list, vk_other_obj_info_list = _vk_load_env_objects_meshes()
    return vk_obj_id_list, vk_rob_link_id_list, vk_other_id_list, vk_other_obj_info_list
 
def _vk_state_setting(vk_particle_cloud, pw_T_camVk_4_4, all_links_info, base_link_info):
    """
    particle setting
    """
    global _vk_context
    vk_state_list = [0] * PARTICLE_NUM
    parNum_times_objNum = PARTICLE_NUM * OBJECT_NUM
    vk_single_obj_state_list = [0] * parNum_times_objNum
    pw_T_camVk_4_4_ = copy.deepcopy(pw_T_camVk_4_4)
    camVk_T_pw_4_4_ = np.linalg.inv(pw_T_camVk_4_4_)
    for index, particle in enumerate(vk_particle_cloud):
        vk_state = vkdepth.State()
        vk_state_list[index] = vk_state
        ## add object in particle
        for obj_index in range(OBJECT_NUM):

            ########### single_obj_index = index * OBJECT_NUM + obj_index
            ########### vk_single_obj_state = vkdepth.State()
            ########### vk_single_obj_state_list[single_obj_index] = vk_single_obj_state

            vk_T_par_pos = copy.deepcopy(particle[obj_index].pos)
            x_pos = vk_T_par_pos[0]
            y_pos = vk_T_par_pos[1]
            z_pos = vk_T_par_pos[2]
            vk_T_par_ori = copy.deepcopy(particle[obj_index].ori)
            x_ori = vk_T_par_ori[0]
            y_ori = vk_T_par_ori[1]
            z_ori = vk_T_par_ori[2]
            w_ori = vk_T_par_ori[3]

            vk_state.add_instance(_vk_obj_id_list[obj_index],
                                  x_pos, y_pos, z_pos,
                                  w_ori, x_ori, y_ori, z_ori) # w, x, y, z
                                  
            ########### vk_single_obj_state.add_instance(_vk_obj_id_list[obj_index],
            ###########                                  x_pos, y_pos, z_pos,
            ###########                                  w_ori, x_ori, y_ori, z_ori) # w, x, y, z
            ########### _vk_context.add_state(vk_single_obj_state)

        # vk mark 
        ## add table/robot... in particle 
        ## all_links_info:
        # There are actually 13 links, of which "link8" and "panda_grasptarget" have no entities.
        ## "link    0,1,2,3,4,5,6,7", "panda_hand", "panda_left_finger", "panda_right_finger"
        # index:    0,1,2,3,4,5,6,7,   9,            10,                  11
        # loop:     0,1,2,3,4,5,6,7,   8,            9,                   10
        # linkstate:x,0,1,2,3,4,5,6,   8,            9,                   10
        all_links_info
        for rob_link_index in range(PANDA_ROBOT_LINK_NUMBER): # 11: [0, 11)
            if rob_link_index == 0:
                vk_T_link_pos = base_link_info[0]
                vk_T_link_ori = base_link_info[1]
            elif rob_link_index < 8 and rob_link_index > 0:
                link_info = all_links_info[rob_link_index-1]
                vk_T_link_pos = link_info[4]
                vk_T_link_ori = link_info[5]
            else:
                link_info = all_links_info[rob_link_index]
                vk_T_link_pos = link_info[4]
                vk_T_link_ori = link_info[5]
            x_pos = vk_T_link_pos[0]
            y_pos = vk_T_link_pos[1]
            z_pos = vk_T_link_pos[2]
            x_ori = vk_T_link_ori[0]
            y_ori = vk_T_link_ori[1]
            z_ori = vk_T_link_ori[2]
            w_ori = vk_T_link_ori[3]
            if rob_link_index == 10:
                y_ori = -y_ori
                z_ori = -z_ori
            vk_state.add_instance(_vk_rob_link_id_list[rob_link_index],
                                    x_pos, y_pos, z_pos,
                                    w_ori, x_ori, y_ori, z_ori) # w, x, y, z

        # other objects
        vk_other_obj_number_ = len(_vk_other_id_list)
        
        for vk_other_obj_index in range(vk_other_obj_number_):
            vk_other_obj_info = _vk_other_obj_info_list[vk_other_obj_index]
            vk_other_obj_name = vk_other_obj_info.obj_name
            vk_other_obj_pos = vk_other_obj_info.pos
            vk_other_obj_ori = vk_other_obj_info.ori
            vk_state.add_instance(_vk_other_id_list[vk_other_obj_index],
                                  vk_other_obj_pos[0], vk_other_obj_pos[1], vk_other_obj_pos[2],
                                  vk_other_obj_ori[3], vk_other_obj_ori[0], vk_other_obj_ori[1], vk_other_obj_ori[2]) # w, x, y, z
            
        _vk_context.add_state(vk_state)
        # vk_state: 
        # 70
        # vk_state->add_instance: 
        # object1, object2, ..., link0, link1, ..., link7, panda_hand, "panda_left_finger, panda_right_finger, table, barrier, ...

    return vk_state_list, vk_single_obj_state_list

# begine to change code 2024.05.31

# get vk rendered depth image
def _vk_depth_image_getting():
    global _vk_context
    vk_rendered_depth_image_array_list = []
    vk_rendered__mask_image_array_list = []
    vk_single_obj_rendered__mask_image_array_list = []
    for par_index in range(PARTICLE_NUM):
        ########### whole_img_index = par_index * (OBJECT_NUM + 1) + OBJECT_NUM

        vk_rendered_depth_image_vkdepth = _vk_context.view(par_index, vkdepth.DEPTH) # <class 'vkdepth.DepthView'>
        vk_rendered__mask_image_vkdepth = _vk_context.view(par_index, vkdepth.MASK) # <class 'vkdepth.DepthView'>
        ########### vk_rendered_depth_image_vkdepth = _vk_context.view(whole_img_index, vkdepth.DEPTH) # <class 'vkdepth.DepthView'>
        ########### vk_rendered__mask_image_vkdepth = _vk_context.view(whole_img_index, vkdepth.MASK) # <class 'vkdepth.DepthView'>

        vk_rendered_depth_image_array = np.array(vk_rendered_depth_image_vkdepth, copy = False) # <class 'numpy.ndarray'>
        vk_rendered__mask_image_array = np.array(vk_rendered__mask_image_vkdepth, copy = False) # <class 'numpy.ndarray'>
        
        vk_rendered_depth_image_array_list.append(vk_rendered_depth_image_array)
        vk_rendered__mask_image_array_list.append(vk_rendered__mask_image_array)

        ########### for obj_index in range(OBJECT_NUM):
        ###########     single_img_index = par_index * (OBJECT_NUM + 1) + obj_index

        ###########     vk_single_obj_rendered__mask_image_vkdepth = _vk_context.view(single_img_index, vkdepth.MASK) # <class 'vkdepth.DepthView'>
        ###########     vk_single_obj_rendered__mask_image_array = np.array(vk_single_obj_rendered__mask_image_vkdepth, copy = False) # <class 'numpy.ndarray'>
        ###########     vk_single_obj_rendered__mask_image_array_list.append(vk_single_obj_rendered__mask_image_array)
    return vk_rendered_depth_image_array_list, vk_rendered__mask_image_array_list, vk_single_obj_rendered__mask_image_array_list

# get rendered depth/seg image model PyBullet
def _vk_get_rendered_depth_image_parallelised(particle_cloud, links_info, base_link_info):
    # vk mark 
    ## Update particle pose->update depth image
    _vk_update_depth_image(_vk_state_list, _vk_single_obj_state_list, particle_cloud, links_info, base_link_info)
    ## Render and Download
    _vk_context.enqueue_render_and_download(vkdepth.DEPTH | vkdepth.MASK)
    ## Waiting for rendering and download
    _vk_context.wait()
    ## Get Depth image
    vk_rendered_depth_image_array_list_, vk_rendered__mask_image_array_list_, vk_single_obj_rendered__mask_image_array_list_ = _vk_depth_image_getting()
    # ============================================================================
    # fig, axs = plt.subplots(1, PARTICLE_NUM)
    # for par_index in range(PARTICLE_NUM):
    #     axs[par_index].imshow(vk_rendered_depth_image_array_list_[par_index])
    # plt.show()
    # for index in range(len(vk_rendered_depth_image_array_list_)):
    #     img_name = "single_obj_Maskimg_"+str(index)+".png"
    #     # cv2.imwrite(os.path.expanduser("~/catkin_ws/src/PBPF/scripts/img_debug/")+real_depth_img_name, (cv_image).astype(np.uint16))
    #     imsave(os.path.expanduser("~/catkin_ws/src/PBPF/scripts/img_debug/")+img_name, vk_rendered_depth_image_array_list_[index], cmap='gray')
    #     # imsave(os.path.expanduser("~/catkin_ws/src/PBPF/scripts/img_debug/")+real_depth_img_name, self.real_depth_image_transferred, cmap='gray')
    # ============================================================================
    return vk_rendered_depth_image_array_list_, vk_rendered__mask_image_array_list_, vk_single_obj_rendered__mask_image_array_list_

# update vk rendered depth image
def _vk_update_depth_image(vk_state_list, vk_single_obj_state_list, vk_particle_cloud, all_links_info, base_link_info):
    for index, particle in enumerate(vk_particle_cloud):
        objs_states = np.array(vk_state_list[index].view(), copy = False)
        for obj_index in range(OBJECT_NUM):
            objs_states[obj_index, 1] = particle[obj_index].pos[0] # x_pos
            objs_states[obj_index, 2] = particle[obj_index].pos[1] # y_pos
            objs_states[obj_index, 3] = particle[obj_index].pos[2] # z_pos
            objs_states[obj_index, 4] = particle[obj_index].ori[3] # w_ori
            objs_states[obj_index, 5] = particle[obj_index].ori[0] # x_ori
            objs_states[obj_index, 6] = particle[obj_index].ori[1] # y_ori
            objs_states[obj_index, 7] = particle[obj_index].ori[2] # z_ori
        for rob_link_index in range(PANDA_ROBOT_LINK_NUMBER):
            if rob_link_index == 0:
                vk_T_link_pos = base_link_info[0]
                vk_T_link_ori = base_link_info[1]
            elif rob_link_index < 8 and rob_link_index > 0:
                link_info = all_links_info[rob_link_index-1]
                vk_T_link_pos = link_info[4] # worldLinkFramePosition
                vk_T_link_ori = link_info[5] # worldLinkFrameOrientation
            else:
                link_info = all_links_info[rob_link_index]
                vk_T_link_pos = link_info[4]
                vk_T_link_ori = link_info[5]
            x_pos = vk_T_link_pos[0]
            y_pos = vk_T_link_pos[1]
            z_pos = vk_T_link_pos[2]
            x_ori = vk_T_link_ori[0]
            y_ori = vk_T_link_ori[1]
            z_ori = vk_T_link_ori[2]
            w_ori = vk_T_link_ori[3]
            if rob_link_index == 10:
                y_ori = -vk_T_link_ori[1]
                z_ori = -vk_T_link_ori[2]
            objs_states[OBJECT_NUM+rob_link_index, 1] = x_pos # x_pos
            objs_states[OBJECT_NUM+rob_link_index, 2] = y_pos # y_pos
            objs_states[OBJECT_NUM+rob_link_index, 3] = z_pos # z_pos
            objs_states[OBJECT_NUM+rob_link_index, 4] = w_ori # w_ori
            objs_states[OBJECT_NUM+rob_link_index, 5] = x_ori # x_ori
            objs_states[OBJECT_NUM+rob_link_index, 6] = y_ori # y_ori
            objs_states[OBJECT_NUM+rob_link_index, 7] = z_ori # z_ori

def visibility_computing_vk(particle_cloud, RGB_weights_lists_):
    _vk_context.enqueue_render_and_download(vkdepth.VISIBILITY)
    _vk_context.wait()
    for index, particle in enumerate(particle_cloud):
        part = _vk_context.part_vis_counts(index)
        part_arr = np.array(part, copy = False)
        full = _vk_context.full_vis_counts(index)
        full_arr = np.array(full, copy = False)
        for obj_index in range(OBJECT_NUM):
            object_name = OBJECT_NAME_LIST[obj_index]
            if full_arr[obj_index] == 0:
                # input()
                weight = RGB_weights_lists_[index][obj_index]
                weight = weight * 0.1
            else:
                visible_score = 1.0 * part_arr[obj_index] / full_arr[obj_index]
                _record_t_visible_score = time.time()
                # _boss_GT_visibility_ADD_df_list[obj_index].loc[_par_panda_step] = [_par_panda_step, _record_t_visible_score - _record_t_begin, visible_score]
                # _boss_GT_visibility_ADD_df_list[obj_index].loc[_par_panda_step] = [_par_panda_step, _record_t_visible_score - _record_t_begin, 0,0,0,0,0,0,0,0,0,0,0,visible_score,0,0,0]

                # weight = particle[obj_index].w
                weight = RGB_weights_lists_[index][obj_index]
                local_obj_visual_by_OBSE_val = global_objects_visual_by_OBSE_list[obj_index]
                local_obj_outlier_by_OBSE_val = global_objects_outlier_by_OBSE_list[obj_index]
                if local_obj_visual_by_OBSE_val==0 and local_obj_outlier_by_OBSE_val==0:
                    # visible_score low, weight low
                    if visible_score < OBJ_VISIBLESCORE_OBSE_WORK_THRESHOLD[object_name]:
                        weight = weight / 3.0
                        # weight = weight * visible_score # for test
                    # visible_score high, weight high
                    else:
                        weight = weight
                else:
                    # visible_score<0.95 low, weight high
                    if OBJ_VISIBLESCORE_OBSE_FAIL_THRESHOLD_LOW[object_name]<=visible_score and visible_score<=OBJ_VISIBLESCORE_OBSE_FAIL_THRESHOLD_HIGH[object_name]:
                        weight = OBJ_VISIBLESCORE_OBSE_FAIL_WEIGHT_HIGH[object_name] * weight
                    else:
                        weight = OBJ_VISIBLESCORE_OBSE_FAIL_WEIGHT_LOW[object_name] * weight # 0.25/0.5
            particle_cloud[index][obj_index].w = weight

    return particle_cloud

# compare depth image
def compare_depth_image_vk_parallelised(real_depth_image_transferred):
    _vk_context.set_reference_image(vkdepth.DEPTH, real_depth_image_transferred)
    _vk_context.enqueue_render_and_download(vkdepth.SCORE)
    _vk_context.wait()
    scoresV = _vk_context.scores()
    scores_0 = np.array( scoresV, copy = False )
    scores_0 = scores_0 / (HEIGHT_DEPTH*WIDTH_DEPTH)
    return scores_0


def create_particles(phy_ENV,
                     pw_T_rob_sim_pose_list_alg, pw_T_obj_obse_obj_list_alg
                     ):
    manager = multiprocessing.Manager()
    single_envs_ = {i: SingleENV(phy_ENV,
                                 pw_T_rob_sim_pose_list_alg, pw_T_obj_obse_obj_list_alg, 
                                 manager.dict()) for i in range(PARTICLE_NUM)}
    for _, single_env in single_envs_.items():
        single_env.start()
        single_env.queue.put((SingleENV.dummy,))
    for _, single_env in single_envs_.items():
        wait_and_get_result_from(single_env)
    print()
    return single_envs_

def wait_and_get_result_from(single_env):
    while True:
        with single_env.lock:
            if len(single_env.result) > 0 and single_env.queue.empty():
                result = single_env.result.copy()
                single_env.result.clear()
                return result
        time.sleep(0.00001)

def get_real_depth_image():
    depth_image_real = ROS_LISTENER.depth_image # persp
    real_depth_image_transferred = depthImageRealTransfer(depth_image_real) # persp
    # real_depth_image_transferred_jax = jnp.array(real_depth_image_transferred) # persp
    return real_depth_image_transferred

def depthImageRealTransfer(depth_image_real):
    cv_image = BRIDGE.imgmsg_to_cv2(depth_image_real,"16UC1")
    cv_image = cv_image / 1000
    return cv_image

def resample_particles_update(particle_cloud, pw_T_obj_obse_objects_pose_list_, D_scores_list_):
    par_num_on_obse = int(math.ceil(PARTICLE_NUM * PICK_PARTICLE_RATE))
    par_num_for_resample = int(PARTICLE_NUM) - int(par_num_on_obse)

    # [[], [], [], ..., []] (PARTICLE_NUM)
    newParticles_list = [[]*OBJECT_NUM for _ in range(PARTICLE_NUM)]

    particles_w = []
    base_w = 0
    base_w_list = []
    base_w_list.append(base_w)
    particle_array_list = []

    # mark
    weight_depth_img_array_ = [1] * PARTICLE_NUM
    if USING_D_FLAG == True:
        if DEPTH_DIFF_VALUE_0_1_FLAG == True:
            # score_that_particle_get: high->high weight; low->low weight
            weight_depth_img_array_ = normalize_score_to_0_1(D_scores_list_)
        else:
            while True:
                print("Not yet implemented")

    for index, particle in enumerate(particle_cloud):
        each_par_weight = 1
        for obj_index in range(OBJECT_NUM):
            each_par_weight = each_par_weight * particle[obj_index].w
        if USING_D_FLAG == True:
            each_par_weight = each_par_weight * weight_depth_img_array_[index]
        particles_w.append(each_par_weight) # to compute the sum
        base_w = base_w + each_par_weight
        base_w_list.append(base_w)
    w_sum = sum(particles_w)
    r = random.uniform(0, w_sum)

    for index in range(par_num_for_resample):
        if w_sum > 0.00000001:
            position = (r + index * w_sum / PARTICLE_NUM) % w_sum
            position_index = computePosition(position, base_w_list)
            particle_array_list.append(position_index)
        else:
            particle_array_list.append(index) # [45, 45, 1, 4, 6, 6, ..., 43]
    index = -1
    for obj_index in range(OBJECT_NUM):
        for index, i in enumerate(particle_array_list): # particle angle 
            particle = Particle(particle_cloud[i][obj_index].par_name,
                                particle_cloud[index][obj_index].visual_par_id,
                                particle_cloud[index][obj_index].no_visual_par_id,
                                particle_cloud[i][obj_index].pos,
                                particle_cloud[i][obj_index].ori,
                                1.0/PARTICLE_NUM, 
                                index,
                                particle_cloud[i][obj_index].linearVelocity,
                                particle_cloud[i][obj_index].angularVelocity)
            newParticles_list[index].append(particle)

        # only work when "local_pick_particle_rate != 0"
        obse_obj_pos = pw_T_obj_obse_objects_pose_list_[obj_index].pos
        obse_obj_ori = pw_T_obj_obse_objects_pose_list_[obj_index].ori # pybullet x,y,z,w
        for index_leftover in range(par_num_on_obse):
            index = index + 1
            particle = Particle(particle_cloud[index_leftover][obj_index].par_name,
                                particle_cloud[index][obj_index].visual_par_id,
                                particle_cloud[index][obj_index].no_visual_par_id,
                                obse_obj_pos,
                                obse_obj_ori,
                                1.0/PARTICLE_NUM, 
                                index,
                                particle_cloud[index_leftover][obj_index].linearVelocity,
                                particle_cloud[index_leftover][obj_index].angularVelocity)
            newParticles_list[index].append(particle)
    return newParticles_list

def normalize_score_to_0_1(score_list):
    par_num_ = len(score_list)
    score_list_min = min(score_list)
    score_list_array_ = np.array(score_list)
    score_list_array_sub = score_list_array_ - score_list_min
    if score_list_array_sub.ndim == 1:
        # print("Dimension of score list is 1")
        pass
    else:
        input("Error: depth_value_difference_list_array_sub.ndim should be 1! Please check the code and press Crtl-C")
    score_list_array_sub_sum = sum(score_list_array_sub)
    if score_list_array_sub_sum == 0:
        score_list_array_sub_sum_over = np.full(par_num_, 1/par_num_)
    else:
        score_list_array_sub_sum_over = score_list_array_sub / score_list_array_sub_sum * 1 # 20
    return score_list_array_sub_sum_over
    # return score_list_array_sub

def computePosition(position, base_w_list):
    for index in range(1, len(base_w_list)):
        if position <= base_w_list[index] and position > base_w_list[index - 1]:
            return index - 1
        else:
            continue

# do not use
def compute_estimate_pos_of_object(particle_cloud): # need to change
    esti_objs_cloud = []
    dis_std_list = []
    ang_std_list = []
    # remenber after resampling weight of each particle is the same
    for obj_index in range(OBJECT_NUM):
        x_set = 0
        y_set = 0
        z_set = 0
        w_set = 0
        quaternions = []
        qws = []
        for index, particle in enumerate(particle_cloud):
            x_set = x_set + particle[obj_index].pos[0] * particle[obj_index].w
            y_set = y_set + particle[obj_index].pos[1] * particle[obj_index].w
            z_set = z_set + particle[obj_index].pos[2] * particle[obj_index].w
            q = quaternion_correction(particle[obj_index].ori)
            qws.append(particle[obj_index].w)
            quaternions.append([q[0], q[1], q[2], q[3]])
            w_set = w_set + particle[obj_index].w
        q = weightedAverageQuaternions(np.array(quaternions), np.array(qws))
        ###################################
        esti_obj_pos_x = x_set/w_set
        esti_obj_pos_y = y_set/w_set
        esti_obj_pos_z = z_set/w_set
        esti_obj_pos = [esti_obj_pos_x, esti_obj_pos_y, esti_obj_pos_z]
        esti_obj_ori_x = q[0]
        esti_obj_ori_y = q[1]
        esti_obj_ori_z = q[2]
        esti_obj_ori_w = q[3]
        esti_obj_ori = [esti_obj_ori_x, esti_obj_ori_y, esti_obj_ori_z, esti_obj_ori_w]
        mean_pose = [esti_obj_pos, esti_obj_ori]
        dis_std, ang_std = compute_std(mean_pose, particle_cloud)
        ###################################
        est_obj_pose = Object_Pose(particle[obj_index].par_name, estimated_object_set[obj_index].obj_id, [x_set/w_set, y_set/w_set, z_set/w_set],  [q[0], q[1], q[2], q[3]], obj_index)
        esti_objs_cloud.append(est_obj_pose)
        dis_std_list.append(dis_std)
        ang_std_list.append(ang_std)
    return esti_objs_cloud, dis_std_list, ang_std_list

def compute_std(mean_pose, particle_cloud):
    mean_pos = copy.deepcopy(mean_pose[0])
    mean_ori = copy.deepcopy(mean_pose[1]) # x,y,z,w
    dis_list = []
    ang_list = []
    for index, particle in enumerate(particle_cloud):
        pos_set = copy.deepcopy(particle[obj_index].pos)
        q = quaternion_correction(particle[obj_index].ori)
        ori_set = copy.deepcopy(q)
        dis_mean_eachPar = compute_pos_err_bt_2_points(pos_set, mean_pos)
        ang_mean_eachPar = compute_ang_err_bt_2_points(ori_set, mean_ori)
        dis_list.append(dis_mean_eachPar)
        ang_list.append(ang_mean_eachPar)
    dis_std = np.std(dis_list)
    ang_std = np.std(ang_list)
    return dis_std, ang_std

def compare_distance_seq(particle_cloud, pw_T_obj_obse_objects_pose_list, visual_by_OBSE_list, outlier_by_OBSE_list):
    weight = 1.0/PARTICLE_NUM
    RGB_weights_lists = [0] * PARTICLE_NUM
    weights_list = [weight] * OBJECT_NUM
    for par_index in range(PARTICLE_NUM):
        RGB_weights_lists[par_index] = weights_list
        for obj_index in range(OBJECT_NUM):
            particle_cloud[par_index][obj_index].w = weight
        # at least one object is detected by camera
    if (sum(visual_by_OBSE_list)<OBJECT_NUM) and (sum(outlier_by_OBSE_list)<OBJECT_NUM):
        for par_index in range(PARTICLE_NUM):
            weight = 1.0/PARTICLE_NUM
            weights_list = [weight] * OBJECT_NUM
            for obj_index in range(OBJECT_NUM):
                weight = 1.0/PARTICLE_NUM
                obj_visual = visual_by_OBSE_list[obj_index]
                obj_outlier = outlier_by_OBSE_list[obj_index]
                if obj_visual==0 and obj_outlier==0:
                    obj_x = particle_cloud[par_index][obj_index].pos[0]
                    obj_y = particle_cloud[par_index][obj_index].pos[1]
                    obj_z = particle_cloud[par_index][obj_index].pos[2]
                    obj_ori = quaternion_correction(particle_cloud[par_index][obj_index].ori)
                    obse_obj_pos = pw_T_obj_obse_objects_pose_list[obj_index].pos
                    obse_obj_ori = pw_T_obj_obse_objects_pose_list[obj_index].ori # pybullet x,y,z,w
                    obse_obj_ori = quaternion_correction(obse_obj_ori)
                    mean = 0
                    # position weight
                    dis_x = abs(obj_x - obse_obj_pos[0])
                    dis_y = abs(obj_y - obse_obj_pos[1])
                    dis_z = abs(obj_z - obse_obj_pos[2])
                    dis_xyz = math.sqrt(dis_x ** 2 + dis_y ** 2 + dis_z ** 2)
                    weight_xyz = normal_distribution(dis_xyz, mean, BOSS_SIGMA_OBS_POS_FOR_RESAMPLE_LIST[obj_index])
                    # rotation weight
                    obse_obj_quat = Quaternion(x=obse_obj_ori[0], y=obse_obj_ori[1], z=obse_obj_ori[2], w=obse_obj_ori[3]) # Quaternion(): w,x,y,z
                    par_quat = Quaternion(x=obj_ori[0], y=obj_ori[1], z=obj_ori[2], w=obj_ori[3])
                    err_bt_par_obse = par_quat * obse_obj_quat.inverse
                    err_bt_par_obse_corr = quaternion_correction([err_bt_par_obse.x, err_bt_par_obse.y, err_bt_par_obse.z, err_bt_par_obse.w])
                    err_bt_par_obse_corr_quat = Quaternion(x=err_bt_par_obse_corr[0], y=err_bt_par_obse_corr[1], z=err_bt_par_obse_corr[2], w=err_bt_par_obse_corr[3]) # Quaternion(): w,x,y,z
                    cos_theta_over_2 = err_bt_par_obse_corr_quat.w
                    sin_theta_over_2 = math.sqrt(err_bt_par_obse_corr_quat.x ** 2 + err_bt_par_obse_corr_quat.y ** 2 + err_bt_par_obse_corr_quat.z ** 2)
                    theta_over_2 = math.atan2(sin_theta_over_2, cos_theta_over_2)
                    theta = theta_over_2 * 2.0
                    weight_ang = normal_distribution(theta, mean, BOSS_SIGMA_OBS_ANG_FOR_RESAMPLE_LIST[obj_index])
                    weight = weight_xyz * weight_ang
                    particle_cloud[par_index][obj_index].w = weight
                    weights_list[obj_index] = weight
                else:
                    particle_cloud[par_index][obj_index].w = weight
                    weights_list[obj_index] = weight
            RGB_weights_lists[par_index] = weights_list
    return RGB_weights_lists, particle_cloud

def normal_distribution(x, mean, sigma):
    return sigma * np.exp(-1*((x-mean)**2)/(2*(sigma**2)))/(math.sqrt(2*np.pi)* sigma)
    
                            


# ctrl-c write down the error file
def signal_handler(sig, frame):

    if RECORD_RESULTS_FLAG == True:
        for obj_index in range(OBJECT_NUM):
            # 70_scene1_rosbag3_repeat0_cracker_time_obse_err_ADD_PBPF_RGBD
            file_save_path = os.path.expanduser('~/catkin_ws/src/PBPF/scripts/results/')
            obj_name = OBJECT_NAME_LIST[obj_index]
            # need to soup
            # if obj_index == 1:
            #     file_name_PBPF_ADD = str(PARTICLE_NUM)+"_scene"+TASK_FLAG+"_rosbag"+str(ROSBAG_TIME)+"_repeat"+str(REPEAT_TIME)+"_"+obj_name+"2_"+UPDATE_STYLE_FLAG+'_PBPF_pose_'+RUNNING_MODEL+'_'+MASS_marker+'_'+FRICTION_marker+'.csv'
            #     file_name_obse_ADD = str(PARTICLE_NUM)+"_scene"+TASK_FLAG+"_rosbag"+str(ROSBAG_TIME)+"_repeat"+str(REPEAT_TIME)+"_"+obj_name+"2_"+UPDATE_STYLE_FLAG+'_obse_pose_'+RUNNING_MODEL+'_'+MASS_marker+'_'+FRICTION_marker+'.csv'
            #     file_name_GT_ADD = str(PARTICLE_NUM)+"_scene"+TASK_FLAG+"_rosbag"+str(ROSBAG_TIME)+"_repeat"+str(REPEAT_TIME)+"_"+obj_name+"2_"+UPDATE_STYLE_FLAG+'_GT_pose_'+RUNNING_MODEL+'_'+MASS_marker+'_'+FRICTION_marker+'.csv'
            # else:
            file_name_PBPF_ADD = str(PARTICLE_NUM)+"_scene"+TASK_FLAG+"_rosbag"+str(ROSBAG_TIME)+"_repeat"+str(REPEAT_TIME)+"_"+obj_name+"_"+UPDATE_STYLE_FLAG+'_PBPF_pose_'+RUNNING_MODEL+'_'+MASS_marker+'_'+FRICTION_marker+'.csv'
            file_name_obse_ADD = str(PARTICLE_NUM)+"_scene"+TASK_FLAG+"_rosbag"+str(ROSBAG_TIME)+"_repeat"+str(REPEAT_TIME)+"_"+obj_name+"_"+UPDATE_STYLE_FLAG+'_obse_pose_'+RUNNING_MODEL+'_'+MASS_marker+'_'+FRICTION_marker+'.csv'
            file_name_GT_ADD = str(PARTICLE_NUM)+"_scene"+TASK_FLAG+"_rosbag"+str(ROSBAG_TIME)+"_repeat"+str(REPEAT_TIME)+"_"+obj_name+"_"+UPDATE_STYLE_FLAG+'_GT_pose_'+RUNNING_MODEL+'_'+MASS_marker+'_'+FRICTION_marker+'.csv'
            # file_name_GT_ADD_V = str(PARTICLE_NUM)+"_scene"+TASK_FLAG+"_rosbag"+str(ROSBAG_TIME)+"_repeat"+str(REPEAT_TIME)+"_"+obj_name+"_"+UPDATE_STYLE_FLAG+'_GT_pose_'+RUNNING_MODEL+'_'+MASS_marker+'_'+FRICTION_marker+'_V.csv'
            
            # if _boss_PBPF_err_ADD_df_list[obj_index].empty:
            #     print(OBJECT_NAME_LIST[obj_index]+" is empty !")
            _boss_PBPF_err_ADD_df_list[obj_index].to_csv(file_save_path+file_name_PBPF_ADD,index=0,header=0,mode='w')
            _boss_obse_err_ADD_df_list[obj_index].to_csv(file_save_path+file_name_obse_ADD,index=0,header=0,mode='w')
            _boss_GT_err_ADD_df_list[obj_index].to_csv(file_save_path+file_name_GT_ADD,index=0,header=0,mode='w')
            # print("write "+obj_name+" PBPF file: "+RUNNING_MODEL)
            # print("write "+obj_name+" obse file: "+RUNNING_MODEL)
            # print("write "+obj_name+" GT file: "+RUNNING_MODEL)

            # _boss_GT_visibility_ADD_df_list[obj_index].to_csv(file_save_path+file_name_GT_ADD_V,index=0,header=0,mode='w')

            # print("write Particle file (should include all objects): "+RUNNING_MODEL)
            for par_index in range(PARTICLE_NUM):
                file_save_path = os.path.expanduser('~/catkin_ws/src/PBPF/scripts/results/particles/'+OBJECT_NAME_LIST[obj_index]+'/')
                # file_save_path = os.path.expanduser('~/catkin_ws/src/PBPF/scripts/results/particles/')
                # need to soup
                file_name_par_ADD = str(PARTICLE_NUM)+"_scene"+TASK_FLAG+"_rosbag"+str(ROSBAG_TIME)+"_repeat"+str(REPEAT_TIME)+"_"+UPDATE_STYLE_FLAG+'_PBPF_pose_'+RUNNING_MODEL+'_'+MASS_marker+'_'+FRICTION_marker+"_"+str(par_index)+'.csv'
                _boss_par_err_ADD_df_list[par_index].to_csv(file_save_path+file_name_par_ADD,index=0,header=0,mode='w')

                
    if RECORD_TIME_CONSUMPTION_FLAG == True:
        OBJECT_NUM
        time_file_save_path = os.path.expanduser('~/catkin_ws/src/PBPF/scripts/results/time_consumption_record/')
        time_file_name = str(OBJECT_NUM)+"_obj_repeat"+str(REPEAT_TIME)+'.csv'
        _boss_time_consumption_df.to_csv(time_file_save_path+time_file_name,index=0,header=0,mode='w')

    
    # print("")
    # print(" -------------------------------------------- ")
    # print("|                                            |")
    # print("|              Thanks for using              |")
    # print("|               our PBPF code!               |")
    # print("|                                            |")
    # print("| Zisong Xu, Rafael Papallas, Jaina Modisett |")
    # print("|    Markus Billeter, and Mehmet R. Dogar    |")
    # print("|          From Universtiy of Leeds          |")
    # print("|                                            |")
    # print(" -------------------------------------------- ")
    sys.exit()

if __name__ == '__main__':
    rospy.init_node('PBPF') # ros node
    signal.signal(signal.SIGINT, signal_handler) # interrupt judgment
    # publish
    pub_ray_trace = rospy.Publisher('/ray_trace_list', particle_list, queue_size = 10)
    ray_trace_list = particle_list()
    pub_par_pose = rospy.Publisher('/par_list', particle_list, queue_size = 10)
    par_list = particle_list()
    rob_pub_par_pose = rospy.Publisher('/rob_par_list', particle_list, queue_size = 10)
    rob_par_list = particle_list()
    pub_esti_pose = rospy.Publisher('/esti_obj_list', estimated_obj_pose, queue_size = 10)
    esti_obj_list = estimated_obj_pose()
    pub_depth_image = rospy.Publisher("/camera/particle_depth_image_converted", Image, queue_size=5)
    # pub_depth_image_list = []
    # for pointcloud_index in range(PARTICLE_NUM):
    #     pub_depth_image = rospy.Publisher("/camera/particle_depth_image_converted_"+str(pointcloud_index), Image, queue_size=5)
    #     pub_depth_image_list.append(pub_depth_image)
    particle_depth_image_converted = Image()
    BRIDGE = CvBridge()
    # only for drawing box
    publish_OBSE_pose_flag = True

    if RUNNING_MODEL == "PBPF_RGB":
        USING_RGB_FLAG = True
        USING_D_FLAG = False
    elif RUNNING_MODEL == "PBPF_RGBD":
        USING_RGB_FLAG = True
        USING_D_FLAG = True
    elif RUNNING_MODEL == "PBPF_D":
        USING_RGB_FLAG = False
        USING_D_FLAG = True
    elif RUNNING_MODEL == "PBPF_Opti":
        USING_RGB_FLAG = True
        USING_D_FLAG = False
    elif RUNNING_MODEL == "PBPF_OptiD":
        USING_RGB_FLAG = True
        USING_D_FLAG = True

    _particle_update_time = 0

    if run_alg_flag == 'CVPF':
        PARTICLE_NUM = 150
    
    # ============================================================================

    # set physics simulation
    if PHYSICS_SIMULATION == "pybullet":
        phy_ENV = PyBulletEnv(parameter_info)
    elif PHYSICS_SIMULATION == "mujoco":
        pass
    else:
        pass
        
    # ============================================================================

    # get camera intrinsic info
    # CAMERA_INFO_TOPIC_COLOR: "/camera/color/camera_info"
    # CAMERA_INFO_TOPIC_DEPTH: "/camera/depth/camera_info"
    camera_intrinsic_parameters_color = _get_camera_intrinsic_params(CAMERA_INFO_TOPIC_COLOR)
    camera_intrinsic_parameters_depth = _get_camera_intrinsic_params(CAMERA_INFO_TOPIC_DEPTH)

    HEIGHT_DEPTH = camera_intrinsic_parameters_depth.image_height # 720/480
    WIDTH_DEPTH = camera_intrinsic_parameters_depth.image_width # 1280/848
    CX_DEPTH = camera_intrinsic_parameters_depth.cx
    CY_DEPTH = camera_intrinsic_parameters_depth.cy
    FX_DEPTH = camera_intrinsic_parameters_depth.fx
    FY_DEPTH = camera_intrinsic_parameters_depth.fy
    
    FOV_H_DEPTH = math.degrees(2 * math.atan(WIDTH_DEPTH / (2*FX_DEPTH))) # fov: horizontal / x
    FOV_V_DEPTH = math.degrees(2 * math.atan(HEIGHT_DEPTH / (2*FY_DEPTH))) # fov: vertical / y
    
    RESOLUTION_DEPTH = (HEIGHT_DEPTH, WIDTH_DEPTH) # 480 848

    # ============================================================================

    pub_OBSE_list = []
    pub_PBPF_list = []
    for obj_index in range(OBJECT_NUM):
        pub_OBSE = rospy.Publisher('DOPE_pose_'+OBJECT_NAME_LIST[obj_index], PoseStamped, queue_size = 1)
        pub_PBPF = rospy.Publisher('PBPF_pose_'+OBJECT_NAME_LIST[obj_index], PoseStamped, queue_size = 1)
        pub_OBSE_list.append(pub_OBSE)
        pub_PBPF_list.append(pub_PBPF)
    
    print("This is "+UPDATE_STYLE_FLAG+" update in scene"+TASK_FLAG)    

    # ============================================================================

    print("Algorithm:",run_alg_flag,"; VERSION:", VERSION, "; RENDER MODEL:", RENDER_FLAG, "; RUNNING MODEL:", RUNNING_MODEL)
    PF_UPDATE_RATE = rospy.Rate(1.0/PF_UPDATE_FREQUENCY[run_alg_flag])
    print("PF UPDATE TIME FREQUENCY:", PF_UPDATE_FREQUENCY[run_alg_flag])

    # ============================================================================

    BOSS_SIGMA_OBS_POS_FOR_RESAMPLE_LIST = [0.05] * OBJECT_NUM
    BOSS_SIGMA_OBS_ANG_FOR_RESAMPLE_LIST = [0.2] * OBJECT_NUM
    for obj_index in range(OBJECT_NUM):
        object_name = OBJECT_NAME_LIST[obj_index]
        BOSS_SIGMA_OBS_POS_FOR_RESAMPLE_LIST[obj_index] = 0.05
        if object_name == "cracker":
            BOSS_SIGMA_OBS_ANG_FOR_RESAMPLE_LIST[obj_index] = 0.6
        else:
            BOSS_SIGMA_OBS_ANG_FOR_RESAMPLE_LIST[obj_index] = 0.2

    # ============================================================================

    time.sleep(0.5)

    # ============================================================================

    ROS_LISTENER = Ros_Listener()
    _tf_listener = tf.TransformListener()
    
    create_scene = Create_Scene(OBJECT_NUM, ROBOT_NUM)
    
    pw_T_rob_sim_pose_list_alg = create_scene.initialize_robot()
    # Here, because we are using only one robot so we use [0]
    _pw_T_rob_sim_4_4 = pw_T_rob_sim_pose_list_alg[0].trans_matrix
    print("========================")
    print("Robot pose in Pybullet world:")
    print(_pw_T_rob_sim_4_4)
    print("========================")
    # get cameraDepth pose
    _pw_T_camD_tf_4_4 = phy_ENV.getCameraInPybulletWorldPose44(_tf_listener, _pw_T_rob_sim_4_4)
    print("========================")
    print("Camera depth len pose in Pybullet world:")
    print(_pw_T_camD_tf_4_4)
    print("========================")
    pw_T_obj_obse_obj_list_alg, trans_ob_list, rot_ob_list = create_scene.initialize_obs_object()
    print("trans_ob_list, rot_ob_list:")
    print(trans_ob_list, rot_ob_list)
    print("========================")
    print("Finish initializing scene")

    # ============================================================================

    # cpu parallel
    _single_envs = create_particles(phy_ENV,
                                    pw_T_rob_sim_pose_list_alg, pw_T_obj_obse_obj_list_alg
                                    )

    _objs_pose_info_list = [0] * PARTICLE_NUM
    _particle_cloud_pub = [0] * PARTICLE_NUM
    t1 = time.time()
    for env_index, single_env in _single_envs.items():
        single_env.queue.put((SingleENV.get_objects_pose, env_index))
    for env_index, single_env in _single_envs.items():  
        objs_pose_info = wait_and_get_result_from(single_env)
        _objs_pose_info_list[env_index] = objs_pose_info
        _particle_cloud_pub[env_index] = objs_pose_info[str(env_index)]
    t2 = time.time()
    # print(t2-t1)

    # ============================================================================

    # get estimated object
    estimated_object_set = _compute_estimate_pos_of_object(_particle_cloud_pub)
    # publish particles/estimated object
    # first publish
    _publish_par_pose_info(_particle_cloud_pub)
    if RECORD_RESULTS_FLAG == True:
        _record_t_PBPF = time.time()
        _record_time_list.append(_record_t_PBPF - _record_t_begin)
    _publish_esti_pose_info(estimated_object_set)

    # convert [obj1, obj2, ...] to list:[[[x,y,z],[x,y,z,w]], [[x,y,z],[x,y,z,w]], ...]
    # estimated_object_set_old = copy.deepcopy(estimated_object_set)
    # estimated_object_set_old_list = process_esti_pose_from_rostopic(estimated_object_set_old)

    # ============================================================================

    if OPTITRACK_FLAG == True and LOCATE_CAMERA_FLAG == "opti": # ar/opti
        realsense_tf = '/RealSense' # (use Optitrack)
    else:
        realsense_tf = '/ar_tracking_camera_frame' # (do not use Optitrack)
    while_loop_time = 0
    print("Locate Camera Method:", realsense_tf)
    while not rospy.is_shutdown():
        while_loop_time =  while_loop_time + 1
        # if while_loop_time > 50:
            # print("WARNING: ")
        if gazebo_flag == True:
            realsense_tf = '/realsense_camera'
        try:
            (trans_camera, rot_camera) = _tf_listener.lookupTransform('/panda_link0', realsense_tf, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    print("Finish getting pose of camera!")
    rob_T_cam_tf_pos = list(trans_camera)
    rob_T_cam_tf_ori = list(rot_camera) # x,y,z,w
    rob_T_cam_tf_ori_anti = [rob_T_cam_tf_ori[3], rob_T_cam_tf_ori[0], rob_T_cam_tf_ori[1], rob_T_cam_tf_ori[2]] # w,x,y,z
    tmp_q = Quaternion(rob_T_cam_tf_ori_anti)
    rob_T_cam_tf_3_3 = tmp_q.rotation_matrix
    rob_T_cam_tf_3_4 = np.c_[rob_T_cam_tf_3_3, rob_T_cam_tf_pos]  # Add position to create 3x4 matrix
    rob_T_cam_tf_4_4 = np.r_[rob_T_cam_tf_3_4, [[0, 0, 0, 1]]]  # Convert to 4x4 homogeneous matrix
    pw_T_cam_tf = np.dot(_pw_T_rob_sim_4_4, rob_T_cam_tf_4_4)
    pw_T_cam_tf_pos = [pw_T_cam_tf[0][3], pw_T_cam_tf[1][3], pw_T_cam_tf[2][3]]
    print("==============================================")
    print("Camera RGB len pose in Robot world:")
    print(rob_T_cam_tf_4_4)
    print("Camera RGB len pose in Pybullet world:")
    print(pw_T_cam_tf)
    print("==============================================")        

    # ============================================================================
            
    # get pose of the end-effector of the robot arm from joints of robot arm 
    phy_ENV.create_new_physicsEnv_Z(pw_T_rob_sim_pose_list_alg)
    phy_ENV.move_rob_in_Z(ROS_LISTENER.current_joint_values)
    rob_link_9_pose_old = phy_ENV.get_rob_end_effector_state_in_Z()

    # ============================================================================

    # initialisation of vk configuration
    if RENDER_FLAG == 'vk':
        print("Begin initializing vulkon...")
        _camD_T_camVk_4_4 = np.array([[1, 0, 0, 0],
                                        [0,-1, 0, 0],
                                        [0, 0,-1, 0],
                                        [0, 0, 0, 1]])
        ## Setup vk_config
        _vk_config = _vk_config_setting()
        ## Setup vk_camera
        _vk_camera, _pw_T_camVk_4_4 = _vk_camera_setting(_pw_T_camD_tf_4_4, _camD_T_camVk_4_4)
        ## create context
        # _vk_context = vkdepth.initialize(_vk_config)
        _vk_context = vkdepth.initialize(_vk_config)
        _vk_context.set_depth_threshold(DEPTH_DIFF_VALUE_0_1_THRESHOLD)
        _vk_context.update_camera(_vk_camera)
        ## Load meshes
        _vk_obj_id_list, _vk_rob_link_id_list, _vk_other_id_list, _vk_other_obj_info_list = _vk_load_meshes()
        ## Create states
        ## state -> particle
        ## instance -> object
        ## if we have many particles we can create many "q = vkdepth.State()"
        _vk_particle_cloud = copy.deepcopy(_particle_cloud_pub)


        all_links_info = phy_ENV.get_all_rob_links_info_in_Z()
        base_link_info = phy_ENV.get_rob_base_state_in_Z()
        _vk_state_list, _vk_single_obj_state_list = _vk_state_setting(_vk_particle_cloud, _pw_T_camVk_4_4, all_links_info, base_link_info)
        ## Render and Download
        _vk_context.enqueue_render_and_download(vkdepth.DEPTH | vkdepth.MASK)
        ## Waiting for rendering and download
        _vk_context.wait()
        ## Get Depth image
        vk_rendered_depth_image_array_list, vk_rendered__mask_image_array_list, vk_single_obj_rendered__mask_image_array_list = _vk_depth_image_getting()

        # for par_index in range(PARTICLE_NUM):
        #     total_elements = vk_rendered__mask_image_array_list[par_index].size
        #     for obj_index in range(OBJECT_NUM):
        #         obj_pixel_num = np.sum(vk_rendered__mask_image_array_list[par_index] == _vk_obj_id_list[obj_index])
        #         single_obj_index = par_index * OBJECT_NUM + obj_index
        #         single_obj_num_zeros = np.sum(vk_single_obj_rendered__mask_image_array_list[single_obj_index] == 0)                    
        #         print(obj_pixel_num, total_elements)
        #         print(single_obj_num_zeros)
        
        # show vk rendered depth image
        if TEST_INITIAL_RENDERED_DEPTH_IMAGE_FLAG == True:
            fig, axs = plt.subplots(2, PARTICLE_NUM)
            for par_index in range(PARTICLE_NUM):
                axs[0, par_index].imshow(vk_rendered_depth_image_array_list[par_index], cmap="gray")
                axs[1, par_index].imshow(vk_rendered__mask_image_array_list[par_index], cmap="gray")
            plt.show()
            # temp_seg_img = copy.deepcopy(vk_rendered__mask_image_array_list[par_index])
            # temp_seg_img[temp_seg_img>100] = 0
            # temp_seg_img[temp_seg_img==1] = 20
            # plt.imshow(temp_seg_img)
            # plt.show()
            # plt.imsave('seg.png', temp_seg_img)
    # ============================================================================

    print("Welcome to Our Approach ! RUNNING MODEL: ", RUNNING_MODEL)

    t_begin = time.time()

    old_obse_time_list = [0] * OBJECT_NUM
    latest_obse_time_list = [0] * OBJECT_NUM
    
    # ============================================================================
    # set parameters
    x_w_list = [0] * OBJECT_NUM
    y_l_list = [0] * OBJECT_NUM
    z_h_list = [0] * OBJECT_NUM
    outlier_dis_list = [0] * OBJECT_NUM
    outlier_ang_list = [0] * OBJECT_NUM
    
    for obj_index in range(OBJECT_NUM):
        object_name = OBJECT_NAME_LIST[obj_index]
        x_w_list[obj_index] = OBJ_SHAPE[object_name]['x_w']
        y_l_list[obj_index] = OBJ_SHAPE[object_name]['y_l']
        z_h_list[obj_index] = OBJ_SHAPE[object_name]['z_h']
        outlier_dis_list[obj_index] = OBJ_OUTLIER_STANDARD[object_name]['dis']
        outlier_ang_list[obj_index] = OBJ_OUTLIER_STANDARD[object_name]['ang']
    # ============================================================================

    while not rospy.is_shutdown():

        global_objects_visual_by_OBSE_list = [0] * OBJECT_NUM
        global_objects_outlier_by_OBSE_list = [0] * OBJECT_NUM

        temp_pw_T_obj_obse_objs_list = []
        temp_pw_T_obj_opti_objs_list = []
        temp_pw_T_obj_opti_objs_list_V = []
        # temp_pw_T_obj_opti_objs_list_V_list = []
        #panda robot moves in the ENV Z
        phy_ENV.move_rob_in_Z(ROS_LISTENER.current_joint_values)

        if RECORD_RESULTS_FLAG == True:
            pw_T_obj_GT_pose = []
        for obj_index in range(OBJECT_NUM):
            object_name = OBJECT_NAME_LIST[obj_index]
            use_gazebo = ""
            if gazebo_flag == True:
                use_gazebo = '_noise'
            try:
                latest_obse_time = _tf_listener.getLatestCommonTime('/panda_link0', '/'+object_name+use_gazebo)
                latest_obse_time_list[obj_index] = latest_obse_time
                
                if (latest_obse_time_list[obj_index].to_sec() > old_obse_time_list[obj_index]):
                    (trans_ob,rot_ob) = _tf_listener.lookupTransform('/panda_link0', '/'+object_name+use_gazebo, rospy.Time(0))
                    global_objects_visual_by_OBSE_list[obj_index] = 0
                    t_after = time.time()
                    trans_ob_list[obj_index] = trans_ob
                    rot_ob_list[obj_index] = rot_ob
                    # print(t_after - t_begin - 14)
                    # print("obse is FRESH:", obj_index)
                else:
                    # obse has not been updating for a while
                    global_objects_visual_by_OBSE_list[obj_index] = 1
                    global_objects_outlier_by_OBSE_list[obj_index] = 1
                    # print("obse is NOT fresh:", obj_index)
                old_obse_time_list[obj_index] = latest_obse_time_list[obj_index].to_sec()
                # break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Main function:", object_name, " can not find TF")
                
            rob_T_obj_obse_pos = list(trans_ob_list[obj_index])
            rob_T_obj_obse_ori = list(rot_ob_list[obj_index]) # x,y,z,W

            rob_T_obj_obse_ori_anti = [rob_T_obj_obse_ori[3], rob_T_obj_obse_ori[0], rob_T_obj_obse_ori[1], rob_T_obj_obse_ori[2]] # w,x,y,z
            tmp_q = Quaternion(rob_T_obj_obse_ori_anti)
            rob_T_obj_obse_3_3 = tmp_q.rotation_matrix
            rob_T_obj_obse_3_4 = np.c_[rob_T_obj_obse_3_3, rob_T_obj_obse_pos]  # Add position to create 3x4 matrix
            rob_T_obj_obse_4_4 = np.r_[rob_T_obj_obse_3_4, [[0, 0, 0, 1]]]  # Convert to 4x4 homogeneous matrix

            pw_T_obj_obse = np.dot(_pw_T_rob_sim_4_4, rob_T_obj_obse_4_4)
            pw_T_obj_obse_pos = [pw_T_obj_obse[0][3], pw_T_obj_obse[1][3], pw_T_obj_obse[2][3]]
            pw_T_obj_obse_ori = transformations.quaternion_from_matrix(pw_T_obj_obse)
            
            pw_T_obj_obse_pose_new = [pw_T_obj_obse_pos, pw_T_obj_obse_ori]

            minDis_obseCur_parOld, minAng_obseCur_parOld = find_closest_pose_metrics(obj_index, _particle_cloud_pub, pw_T_obj_obse_pose_new)            

            if run_alg_flag == "PBPF":
                if minDis_obseCur_parOld > outlier_dis_list[obj_index] or minAng_obseCur_parOld > outlier_ang_list[obj_index]:
                    global_objects_outlier_by_OBSE_list[obj_index] = 1
                else:
                    assert run_alg_flag == "PBPF", "Not Done"
            else:
                assert run_alg_flag == "PBPF", "Not Done"
            # only for drawing BOX/ need to change
            if publish_OBSE_pose_flag == True:
                pose_OBSE = PoseStamped()
                pose_OBSE.pose.position.x = pw_T_obj_obse_pos[0]
                pose_OBSE.pose.position.y = pw_T_obj_obse_pos[1]
                pose_OBSE.pose.position.z = pw_T_obj_obse_pos[2]
                pose_OBSE.pose.orientation.x = pw_T_obj_obse_ori[0]
                pose_OBSE.pose.orientation.y = pw_T_obj_obse_ori[1]
                pose_OBSE.pose.orientation.z = pw_T_obj_obse_ori[2]
                pose_OBSE.pose.orientation.w = pw_T_obj_obse_ori[3]
                pub_OBSE_list[obj_index].publish(pose_OBSE)
            pw_T_obj_obse_name = object_name
            pw_T_obj_obse_id = 0
            obse_object = Object_Pose(pw_T_obj_obse_name, pw_T_obj_obse_id, pw_T_obj_obse_pos, pw_T_obj_obse_ori, index=obj_index)
            temp_pw_T_obj_obse_objs_list.append(obse_object)

            if RECORD_RESULTS_FLAG == True:
                # Only for recording errors
                obj_name = OBJECT_NAME_LIST[obj_index]
                opti_T_rob_opti_pos = ROS_LISTENER.listen_2_robot_pose()[0]
                opti_T_rob_opti_ori = ROS_LISTENER.listen_2_robot_pose()[1]
                # pose of objects in OptiTrack coordinate frame
                # if (obj_name == 'cracker') or (obj_name == 'Parmesan') or (obj_name == 'SaladDressing'):
                #     opti_T_obj_opti_pos = ROS_LISTENER.listen_2_object_pose('soup')[0]
                #     opti_T_obj_opti_ori = ROS_LISTENER.listen_2_object_pose('soup')[1]
                # else:
                opti_T_obj_opti_pos = ROS_LISTENER.listen_2_object_pose(obj_name)[0]
                opti_T_obj_opti_ori = ROS_LISTENER.listen_2_object_pose(obj_name)[1]
                # pose of objects in robot coordinate frame
                rob_T_obj_opti_4_4 = compute_transformation_matrix(opti_T_rob_opti_pos, opti_T_rob_opti_ori, opti_T_obj_opti_pos, opti_T_obj_opti_ori)
                pw_T_obj_opti_4_4 = np.dot(_pw_T_rob_sim_4_4, rob_T_obj_opti_4_4)
                pw_T_obj_opti_pos = [pw_T_obj_opti_4_4[0][3], pw_T_obj_opti_4_4[1][3], pw_T_obj_opti_4_4[2][3]]
                pw_T_obj_opti_ori = transformations.quaternion_from_matrix(pw_T_obj_opti_4_4)
                # print(pw_T_obj_opti_pos, pw_T_obj_opti_ori)
                pw_T_obj_GT_pose.append(pw_T_obj_opti_4_4)
                
                obj = obj_name
                scene = "scene"+str(TASK_FLAG)
                obj_scene = obj_name+"_scene"+str(TASK_FLAG)
                _record_t = time.time()
                # x, y, z ,w
                _boss_GT_err_ADD_df_list[obj_index].loc[_GT_panda_step] = [_GT_panda_step, _record_t - _record_t_begin, pw_T_obj_opti_pos[0], pw_T_obj_opti_pos[1], pw_T_obj_opti_pos[2], pw_T_obj_opti_ori[0], pw_T_obj_opti_ori[1], pw_T_obj_opti_ori[2], pw_T_obj_opti_ori[3], 'GT', obj, scene, PARTICLE_NUM, VERSION, obj_name, MASS_marker, FRICTION_marker]
                _boss_obse_err_ADD_df_list[obj_index].loc[_obse_panda_step] = [_obse_panda_step, _record_t - _record_t_begin, pw_T_obj_obse_pos[0], pw_T_obj_obse_pos[1], pw_T_obj_obse_pos[2], pw_T_obj_obse_ori[0], pw_T_obj_obse_ori[1], pw_T_obj_obse_ori[2], pw_T_obj_obse_ori[3], 'OBSE', obj, scene, PARTICLE_NUM, VERSION, obj_name, MASS_marker, FRICTION_marker]      

                opti_object = Object_Pose(obj_name, pw_T_obj_obse_id, pw_T_obj_opti_pos, pw_T_obj_opti_ori, index=obj_index)
                temp_pw_T_obj_opti_objs_list.append(opti_object)

                opti_object_V = Particle(obj_name, 0, pw_T_obj_obse_id, pw_T_obj_opti_pos, pw_T_obj_opti_ori, 1, 0, 0, 0)
                temp_pw_T_obj_opti_objs_list_V.append(opti_object_V)
                # if obj_index == 0:
                #     print(pw_T_obj_obse_pos[1])

        _GT_panda_step = _GT_panda_step + 1
        _obse_panda_step = _obse_panda_step + 1

        pw_T_obj_obse_objects_list = copy.deepcopy(temp_pw_T_obj_obse_objs_list)
        pw_T_obj_opti_objects_list = copy.deepcopy(temp_pw_T_obj_opti_objs_list)

        
        if RECORD_RESULTS_FLAG == True:
            _record_obse_pose_first_flag = 1
            _record_obse_pose_list.append(pw_T_obj_obse_objects_list)
            _record_GT_pose_list.append(pw_T_obj_GT_pose)

        # compute distance between old robot and cur robot (position and angle)
        rob_link_9_pose_cur = phy_ENV.get_rob_end_effector_state_in_Z()
        tmp_quat = rob_link_9_pose_cur[1] # x,y,z,w
        tmp_rot = R.from_quat(tmp_quat)
        rob_link_9_ang_cur = tmp_rot.as_euler('xyz', degrees=False)
        rob_link_9_ang_cur = tuple(rob_link_9_ang_cur)
        dis_robcur_robold = compute_pos_err_bt_2_points(rob_link_9_pose_cur[0], rob_link_9_pose_old[0])
                
        # update according to the pose
        if UPDATE_STYLE_FLAG == "pose":
            while True:
                print("Not yet implemented")
        # update according to the time
        elif UPDATE_STYLE_FLAG == "time":
            while not rospy.is_shutdown():
                _contact_results_list = [False] * PARTICLE_NUM
                particles_pose_list = []
                t_begin_sleep = time.time()
                Only_update_robot_flag = False
                if run_alg_flag == "PBPF": # PBPF algorithm
                    # check robot arm and objects have collision
                    for env_index, single_env in _single_envs.items():
                        single_env.queue.put((SingleENV.isAnyParticleInContact, ))
                    for env_index, single_env in _single_envs.items():
                        contact_result = wait_and_get_result_from(single_env)
                        _contact_results_list[env_index] = contact_result
                    if any(result['result'] for result in _contact_results_list) and (dis_robcur_robold > 0.002):
                    # if True:
                        t_begin_PBPF = time.time()
                        simRobot_touch_par_flag = 1
                        _particle_update_time = _particle_update_time + 1
                        if PRINT_FLAG == True:
                            print("Run ", RUNNING_MODEL, "; Repeat time:", REPEAT_TIME, "; Particle Update Time:", _particle_update_time)

                        _pw_T_obj_obse_objects_pose_list = copy.deepcopy(pw_T_obj_obse_objects_list)
                        _pw_T_obj_opti_objects_pose_list = copy.deepcopy(pw_T_obj_opti_objects_list)
                        # _pw_T_obj_opti_objects_pose_list_V = copy.deepcopy(temp_pw_T_obj_opti_objs_list_V_list)
                        
                        # execute PBPF algorithm movement
                        if PRINT_FLAG == True:
                            pass
                        if RUNNING_MODEL == "PBPF_Opti" or RUNNING_MODEL == "PBPF_OptiD":
                        # if RUNNING_MODEL == "PBPF_Opti" or RUNNING_MODEL == "PBPF_OptiD" or RUNNING_MODEL == "PBPF_RGBD":
                            global_objects_visual_by_OBSE_list = [0] * OBJECT_NUM
                            global_objects_outlier_by_OBSE_list = [0] *OBJECT_NUM

                        print("-------------------------------------")
                        print("global_objects_visual_by_OBSE_list: -")
                        print(global_objects_visual_by_OBSE_list)
                        print("global_objects_outlier_by_OBSE_list:-")
                        print(global_objects_outlier_by_OBSE_list)
                        print("-------------------------------------")


                        # I. Motion Model #########################################################################################
                        t_before_motion_model = time.time()
                        for env_index, single_env in _single_envs.items():
                            single_env.queue.put((SingleENV.motion_model, ROS_LISTENER.current_joint_values, env_index))
                        for env_index, single_env in _single_envs.items():
                            objs_pose_info = wait_and_get_result_from(single_env)
                            _objs_pose_info_list[env_index] = objs_pose_info
                            _particle_cloud_pub[env_index] = objs_pose_info[str(env_index)]
                        t_after_motion_model = time.time()
                        Motion_model_time_consumption = t_after_motion_model - t_before_motion_model
                        Motion_model_time_consuming_list.append(Motion_model_time_consumption)
                        if PRINT_FLAG == True:
                            print("--------------------------------------------------------")
                            print("Motion model cost time:", Motion_model_time_consumption)
                            print("--------------------------------------------------------")


                        # II. Observation Model ###################################################################################
                        # A. observation model (DEPTH)
                        _D_scores_list = []
                        t_before_observation_model = time.time()
                        # if USING_D_FLAG == True:
                        # _real_depth_image_transferred, _real_depth_image_transferred_jax = get_real_depth_image()
                        _real_depth_image_transferred = get_real_depth_image()
                        # a. Render Depth Image ###############################################################################
                        t_before_render = time.time()
                        if RENDER_FLAG == 'vk':
                            # get robot links pose
                            for env_index, single_env in _single_envs.items():
                                single_env.queue.put((SingleENV.getLinkStates, ))
                            for env_index, single_env in _single_envs.items():
                                _links_info = wait_and_get_result_from(single_env)
                            # here, all the links_info should be the same, so I can only get the last one!
                            _links_info = _links_info["links_info"]

                            base_link_info = phy_ENV.get_rob_base_state_in_Z()
                            _vk_get_rendered_depth_image_parallelised(_particle_cloud_pub, _links_info, base_link_info)
                            

                        t_after_render = time.time()
                        Render_depth_image_time_consumption = t_after_render - t_before_render
                        Render_depth_image_time_comsuming_list.append(Render_depth_image_time_consumption)
                        if PRINT_FLAG == True:
                            print("--------------------------------------------------------")
                            print("Render cost time:", Render_depth_image_time_consumption)
                            print("--------------------------------------------------------")
                        # b. Compare Depth Image ############################################################################## 
                        t_before_compare = time.time()
                        if COMPARE_DEPTH_IMG_VK == True:
                            _D_scores_list = compare_depth_image_vk_parallelised(_real_depth_image_transferred)
                            if USING_D_FLAG != True:
                                _D_scores_list = []
                        t_after_compare = time.time()
                        Compare_depth_image_time_consumption = t_after_compare - t_before_compare
                        Compare_depth_image_time_consuming_list.append(Compare_depth_image_time_consumption)
                        if PRINT_FLAG == True:
                            print("--------------------------------------------------------")
                            print("Compare image cost time:", Compare_depth_image_time_consumption)
                            print("--------------------------------------------------------")

                        # B. observation model (RGB)
                        _RGB_weights_lists = [0] * PARTICLE_NUM
                        if USING_RGB_FLAG == True:
                            # a. Compare Distance #################################################################################
                            t_before_RGB = time.time()
                            compare_distance_method = "seq" # seq/multi
                            if compare_distance_method == "seq":
                                if RUNNING_MODEL == "PBPF_Opti" or RUNNING_MODEL == "PBPF_OptiD": # need to have ground truth
                                    _RGB_weights_lists, test_particle_cloud_pub = compare_distance_seq(_particle_cloud_pub, _pw_T_obj_opti_objects_pose_list, global_objects_visual_by_OBSE_list, global_objects_outlier_by_OBSE_list)
                                else:
                                    _RGB_weights_lists, test_particle_cloud_pub = compare_distance_seq(_particle_cloud_pub, _pw_T_obj_obse_objects_pose_list, global_objects_visual_by_OBSE_list, global_objects_outlier_by_OBSE_list)
                            elif compare_distance_method == "multi":
                                for env_index, single_env in _single_envs.items():
                                    single_env.queue.put((SingleENV.compare_distance, env_index, _pw_T_obj_obse_objects_pose_list, global_objects_visual_by_OBSE_list, global_objects_outlier_by_OBSE_list))
                                for env_index, single_env in _single_envs.items():
                                    _RGB_weights_list = wait_and_get_result_from(single_env)
                                    _RGB_weights_lists[env_index] = _RGB_weights_list[str(env_index)]
                            t_after_RGB = time.time()
                            Compare_distance_time_consumption = t_after_RGB - t_before_RGB
                            Compare_distance_time_consuming_list.append(Compare_distance_time_consumption)
                            if PRINT_FLAG == True:
                                print("--------------------------------------------------------")
                                print("Compare distance cost time:", Compare_distance_time_consumption)
                                print("--------------------------------------------------------")
                            # b. Visibility Score #################################################################################
                            t_before_Vis = time.time()

                            if RUNNING_MODEL == "PBPF_Opti" or RUNNING_MODEL == "PBPF_OptiD":
                            # if RUNNING_MODEL == "PBPF_Opti" or RUNNING_MODEL == "PBPF_OptiD" or RUNNING_MODEL == "PBPF_RGBD":
                                pass
                            else:
                                if VISIBILITY_COMPUTE_VK == True:
                                    new_particle_cloud = visibility_computing_vk(_particle_cloud_pub, _RGB_weights_lists)
                                    _particle_cloud_pub = copy.deepcopy(new_particle_cloud)
                                else:
                                    while True:
                                        print("Not yet implemented")
                            t_after_Vis = time.time()
                            Compute_visibility_score_time_consumption = t_after_Vis - t_before_Vis
                            Compute_visibility_score_time_consuming_list.append(Compute_visibility_score_time_consumption)
                            if PRINT_FLAG == True:
                                print("--------------------------------------------------------")
                                print("Compute visibility score cost time:", Compute_visibility_score_time_consumption)
                                print("--------------------------------------------------------")
                            t_after_Vis = time.time()
                        t_after_observation_model = time.time()
                        Observation_model_time_consumption = t_after_observation_model - t_before_observation_model
                        Observation_model_time_consuming_list.append(Observation_model_time_consumption)
                        if PRINT_FLAG == True:
                            print("--------------------------------------------------------")
                            print("Observation model cost time:", Observation_model_time_consumption)
                            print("--------------------------------------------------------")
                        

                        # III. Resampling #########################################################################################
                        t_before_resample = time.time()
                        new_particle_cloud = resample_particles_update(_particle_cloud_pub, _pw_T_obj_obse_objects_pose_list, _D_scores_list)
                        t_after_resample = time.time()
                        Resample_time_consumption = t_after_resample - t_before_resample
                        Resample_time_consuming_list.append(Resample_time_consumption)
                        if PRINT_FLAG == True:
                            print("Resample/Compute Mean/... cost time:", t_after_resample - t_before_resample)
                        _particle_cloud_pub = copy.deepcopy(new_particle_cloud)
                        t_after_deepcopy = time.time()
                        Deepcopy_time_consumption = t_after_deepcopy - t_after_resample
                        Deepcopy_time_consuming_list.append(Deepcopy_time_consumption)

                        for env_index, single_env in _single_envs.items():
                            single_env.queue.put((SingleENV.set_particle_in_each_sim_env, _particle_cloud_pub[env_index]))
                            # single_env.queue.put((SingleENV.set_particle_in_each_sim_env, _pw_T_obj_opti_objects_pose_list_V[env_index]))
                        for env_index, single_env in _single_envs.items():
                            _empty_return = wait_and_get_result_from(single_env)
                        _publish_par_pose_info(_particle_cloud_pub)
                        # _publish_par_pose_info(_pw_T_obj_opti_objects_pose_list_V)
                        estimated_object_set = _compute_estimate_pos_of_object(_particle_cloud_pub)
                        # estimated_object_set = _compute_estimate_pos_of_object(_pw_T_obj_opti_objects_pose_list_V)
                        _publish_esti_pose_info(estimated_object_set)
                        t_after_setpub = time.time()
                        Setpub_time_consumption = t_after_setpub - t_after_deepcopy
                        Setpub_time_consuming_list.append(Setpub_time_consumption)

                        if RECORD_RESULTS_FLAG == True:
                            _record_obse_pose_list.append(_pw_T_obj_obse_objects_pose_list)
                            _record_GT_pose_list.append(pw_T_obj_GT_pose)
                            _record_t_PBPF = time.time()
                            _record_time_list.append(_record_t_PBPF - _record_t_begin)
                        if SHOW_RAY == True:
                            while True:
                                print("Not yet implemented")
                        ###########################################################################################################
                        ###########################################################################################################
                        ################################## One Particle Filtering Update Finished #################################
                        ###########################################################################################################
                        ###########################################################################################################
                        rob_link_9_pose_old = copy.deepcopy(rob_link_9_pose_cur)
                        t_finish_PBPF = time.time()
                        One_update_time_consumption = t_finish_PBPF - t_begin_PBPF
                        One_update_time_cosuming_list.append(One_update_time_consumption)
                        if PRINT_FLAG == True:
                            print("Time consuming:", One_update_time_consumption)
                            print("Mean value:", np.mean(One_update_time_cosuming_list))
                        simRobot_touch_par_flag = 0
                        _record_time_consumption = time.time()    
                        _boss_time_consumption_df.loc[_time_record_index] = [_time_record_index, _record_time_consumption - _record_t_begin, 
                                                                             Motion_model_time_consumption, 
                                                                             Render_depth_image_time_consumption, Compare_depth_image_time_consumption, 
                                                                             Compare_distance_time_consumption, Compute_visibility_score_time_consumption,
                                                                             Resample_time_consumption, Deepcopy_time_consumption, Setpub_time_consumption, Observation_model_time_consumption,
                                                                             One_update_time_consumption]     
                        _time_record_index = _time_record_index + 1

                    else:
                        Only_update_robot_flag = True
                        for env_index, single_env in _single_envs.items():
                            single_env.queue.put((SingleENV.move_robot_JointPosition, ROS_LISTENER.current_joint_values))
                        for env_index, single_env in _single_envs.items():
                            _empty_return = wait_and_get_result_from(single_env)
                        # get particles pose
                        for env_index, single_env in _single_envs.items():
                            single_env.queue.put((SingleENV.get_objects_pose, env_index))
                        for env_index, single_env in _single_envs.items():  
                            objs_pose_info = wait_and_get_result_from(single_env)
                            _objs_pose_info_list[env_index] = objs_pose_info
                            _particle_cloud_pub[env_index] = objs_pose_info[str(env_index)]
                        _publish_par_pose_info(_particle_cloud_pub)

                estimated_object_set_old = copy.deepcopy(estimated_object_set)
                estimated_object_set_old_list = process_esti_pose_from_rostopic(estimated_object_set_old)
                
                if Only_update_robot_flag == False:
                    print("Waiting for next loop")
                    PF_UPDATE_RATE.sleep()
                    t_finish_sleep = time.time()
                    print("sleep time:", t_finish_sleep - t_begin_sleep)
                    print("========================================")
                break    
        t_end_while = time.time()




