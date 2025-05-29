#!/usr/bin/python3

# pybullet
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client as bc
#
import math
import random
import os
import numpy as np
# 
from pyquaternion import Quaternion
#
from PhysicsEnv import PhysicsEnv
from quaternion_averaging import weightedAverageQuaternions
from Particle import Particle
from Object_Pose import Object_Pose
from PhysicsEnv import PhysicsEnv
# ros
import tf
import tf.transformations as transformations
import rospy

class PyBulletEnv(PhysicsEnv):
    def __init__(self, parameter_info, **kwargs):
        
        # ============================================================================

        self.parameter_info = parameter_info
        
        # ============================================================================

        self.RUN_ALG_FLAG = self.parameter_info['run_alg_flag']
        
        self.PHYSICS_SIMULATION = self.parameter_info['physics_simulation']
        self.update_style_flag = self.parameter_info['update_style_flag']
        self.SIM_TIME_STEP = self.parameter_info['sim_time_step']
        self.PF_UPDATE_FREQUENCY = self.parameter_info['pf_update_frequency']
    
        self.PF_UPDATE_INTERVAL_IN_SIM = self.PF_UPDATE_FREQUENCY[self.RUN_ALG_FLAG] / self.SIM_TIME_STEP
        
        self.OPTITRACK_FLAG = self.parameter_info['optitrack_flag']
        self.GAZEBO_FLAG = self.parameter_info['gazebo_flag']
        self.LOCATE_CAMERA_FLAG = self.parameter_info['locate_camera_flag']
        self.TASK_FLAG = self.parameter_info['task_flag']
        self.SIM_REAL_WORLD_FLAG = self.parameter_info['sim_real_world_flag']
        self.SHOW_RAY = self.parameter_info['show_ray'] 
        self.OBJECT_NAME_LIST = self.parameter_info['object_name_list']
        self.OBJECT_NUM = self.parameter_info['object_num']
        self.PARTICLE_NUM = self.parameter_info['particle_num']

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

        self.collision_detection_obj_id_collection = []
        self.particle_objects_id_collection = ["None"] * self.OBJECT_NUM
        self.objects_list = ["None"] * self.OBJECT_NUM

        # ============================================================================

    def init_env(self, pw_T_rob_sim_pose_list_alg, pw_T_obj_obse_obj_list_alg):

        self.pw_T_rob_sim_pose_list_alg = pw_T_rob_sim_pose_list_alg
        self.pw_T_obj_obse_obj_list_alg = pw_T_obj_obse_obj_list_alg

        if self.SHOW_RAY == True:
            self.p_env = bc.BulletClient(connection_mode=p.GUI_SERVER) # DIRECT,GUI_SERVER
        else:
            self.p_env = bc.BulletClient(connection_mode=p.DIRECT) # DIRECT,GUI_SERVER
        if self.update_style_flag == "time":
            self.p_env.setTimeStep(self.SIM_TIME_STEP)
        else:
            pass # 
        self.p_env.resetDebugVisualizerCamera(cameraDistance=1., cameraYaw=90, cameraPitch=-50, cameraTargetPosition=[0.1,0.15,0.35])  
        self.p_env.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p_env.setGravity(0, 0, -9.81)
        self.p_env.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
        
        self.add_robot()
        self.add_static_obstacles()
        self.add_target_objects()
    
    def add_static_obstacles(self):
        plane_id = self.p_env.loadURDF("plane.urdf")
        if self.TASK_FLAG == "1":
            pw_T_pringles_pos = [0.6652218209791124, 0.058946644391304814, 0.8277292172960276]
            pw_T_pringles_ori = [ 0.67280124, -0.20574896, -0.20600051, 0.68012472] # x, y, z, w
            pringles_id = self.p_env.loadURDF(os.path.expanduser("~/project/object/others/pringles.urdf"),
                                              pw_T_pringles_pos, pw_T_pringles_ori, useFixedBase=1)

        if self.SIM_REAL_WORLD_FLAG == True:
            table_pos_1 = [0.46, -0.01, 0.702] # 0.710
            table_ori_1 = self.p_env.getQuaternionFromEuler([0,0,0])
            self.table_id_1 = self.p_env.loadURDF(os.path.expanduser("~/project/object/others/table.urdf"), table_pos_1, table_ori_1, useFixedBase = 1)


            barry_pos_1 = [-0.694, 0.443, 0.895]
            barry_ori_1 = self.p_env.getQuaternionFromEuler([0,math.pi/2,0])
            barry_id_1 = self.p_env.loadURDF(os.path.expanduser("~/project/object/others/barrier.urdf"), barry_pos_1, barry_ori_1, useFixedBase = 1)
            
            barry_pos_2 = [-0.694, -0.607, 0.895]
            barry_ori_2 = self.p_env.getQuaternionFromEuler([0,math.pi/2,0])
            barry_id_2 = self.p_env.loadURDF(os.path.expanduser("~/project/object/others/barrier.urdf"), barry_pos_2, barry_ori_2, useFixedBase = 1)

            barry_pos_3 = [0.459, -0.972, 0.895]
            barry_ori_3 = self.p_env.getQuaternionFromEuler([0,math.pi/2,math.pi/2])
            barry_id_3 = self.p_env.loadURDF(os.path.expanduser("~/project/object/others/barrier.urdf"), barry_pos_3, barry_ori_3, useFixedBase = 1)

            board_pos_1 = [0.274, 0.581, 0.87575]
            board_ori_1 = self.p_env.getQuaternionFromEuler([math.pi/2,math.pi/2,0])
            self.board_id_1 = self.p_env.loadURDF(os.path.expanduser("~/project/object/others/board.urdf"), board_pos_1, board_ori_1, useFixedBase = 1)
            self.collision_detection_obj_id_collection.append(self.board_id_1)
        else:
            pass

    def add_robot(self):
        real_robot_start_pos = self.pw_T_rob_sim_pose_list_alg[0].pos
        real_robot_start_ori = self.pw_T_rob_sim_pose_list_alg[0].ori
        joint_of_robot = self.pw_T_rob_sim_pose_list_alg[0].joints
        self.robot_id = self.p_env.loadURDF(os.path.expanduser("~/project/data/bullet3-master/examples/pybullet/gym/pybullet_data/franka_panda/panda.urdf"),
                                            real_robot_start_pos, real_robot_start_ori, useFixedBase=1)
        self.init_set_sim_robot_JointPosition(joint_of_robot)
        self.collision_detection_obj_id_collection.append(self.robot_id)

    def add_target_objects(self):
        print_object_name_flag = 0
        for obj_index in range(self.OBJECT_NUM):
            obj_obse_pos = self.pw_T_obj_obse_obj_list_alg[obj_index].pos
            obj_obse_ori = self.pw_T_obj_obse_obj_list_alg[obj_index].ori
            obj_obse_name = self.pw_T_obj_obse_obj_list_alg[obj_index].obj_name
            if print_object_name_flag == obj_index:
                print_object_name_flag = print_object_name_flag + 1
            particle_pos, particle_ori = self.generate_random_pose(obj_obse_pos, obj_obse_ori, 
                                                                   self.sigma_obs_x_for_init, self.sigma_obs_y_for_init, self.sigma_obs_z_for_init,
                                                                   self.sigma_obs_ang_for_init)
            gazebo_contain = ""
            if self.GAZEBO_FLAG == True:
                gazebo_contain = "gazebo_"
            if obj_obse_name == "soup2":
                obj_obse_name = "soup"
            particle_no_visual_id = self.p_env.loadURDF(os.path.expanduser("~/project/object/"+gazebo_contain+obj_obse_name+"/"+gazebo_contain+obj_obse_name+"_par_no_visual_hor.urdf"),
                                                        particle_pos, particle_ori)
            self.collision_detection_obj_id_collection.append(particle_no_visual_id)
            self.particle_objects_id_collection[obj_index] = particle_no_visual_id

            conter = 0
            while True:
                flag = 0
                conter = conter + 1
                length_collision_detection_obj_id = len(self.collision_detection_obj_id_collection)
                for check_num in range(length_collision_detection_obj_id-1):
                    self.p_env.stepSimulation()
                    contacts = self.p_env.getContactPoints(bodyA=self.collision_detection_obj_id_collection[check_num], 
                                                           bodyB=self.collision_detection_obj_id_collection[-1])
                    for contact in contacts:
                        contactNormalOnBtoA = contact[7]
                        contact_dis = contact[8]
                        if contact_dis < -0.001:
                            par_x_ = particle_pos[0] + contactNormalOnBtoA[0]*contact_dis/2
                            par_y_ = particle_pos[1] + contactNormalOnBtoA[1]*contact_dis/2
                            par_z_ = particle_pos[2] + contactNormalOnBtoA[2]*contact_dis/2
                            particle_pos = [par_x_, par_y_, par_z_]
                            if conter > 20:
                                print("init more than 20 times")
                                conter = 0
                                particle_pos, particle_ori = self.generate_random_pose(obj_obse_pos, obj_obse_ori, 
                                                                                       self.sigma_obs_x_for_init, self.sigma_obs_y_for_init, self.sigma_obs_z_for_init,
                                                                                       self.sigma_obs_ang_for_init)
                            self.p_env.resetBasePositionAndOrientation(particle_no_visual_id, particle_pos, particle_ori)
                            flag = 1
                            break
                    if flag == 1:
                        break
                if flag == 0:
                    break
            objPose = Particle(obj_obse_name, 0, particle_no_visual_id, particle_pos, particle_ori, 1/self.PARTICLE_NUM, 0, 0, 0)
            self.objects_list[obj_index] = objPose




    def get_objects_pose(self, par_index):
        # for time_index in range(int(self.PF_UPDATE_INTERVAL_IN_SIM)):
        #     self.p_env.stepSimulation()
        return_results = []
        for obj_index in range(self.OBJECT_NUM):
            obj_id = self.particle_objects_id_collection[obj_index]
            obj_info = self.p_env.getBasePositionAndOrientation(obj_id)    
            obj_name = self.OBJECT_NAME_LIST[obj_index]
            obj_tuple = (obj_name, obj_info)
            return_results.append(obj_tuple)
            pos_ = obj_info[0]
            ori_ = obj_info[1]
            self.objects_list[obj_index].pos = [pos_[0], pos_[1], pos_[2]]
            self.objects_list[obj_index].ori = [ori_[0], ori_[1], ori_[2], ori_[3]] # x, y, z, w
        return_results.append((str(par_index), self.objects_list))
        # self.p_env.disconnect()
        return return_results

    def isAnyParticleInContact(self):
        for obj_index in range(self.OBJECT_NUM):
            # get object ID
            obj_id = self.particle_objects_id_collection[obj_index]
            # check contact 
            pmin, pmax = self.p_env.getAABB(obj_id)
            collide_ids = self.p_env.getOverlappingObjects(pmin, pmax)
            length = len(collide_ids)
            for t_i in range(length):
                if collide_ids[t_i][1] == 8 or collide_ids[t_i][1] == 9 or collide_ids[t_i][1] == 10 or collide_ids[t_i][1] == 11:
                    return [('result', True)]
        return [('result', False)]

    def motion_model(self, joint_states, par_index):
        # change object parameters
        collision_detection_obj_id_ = []
        return_results = []
        for obj_index in range(self.OBJECT_NUM):
            obj_id = self.objects_list[obj_index].no_visual_par_id
            # self.p_env.resetBaseVelocity(obj_id, 0, 0)
            self.p_env.resetBaseVelocity(obj_id,
                                         self.objects_list[obj_index].linearVelocity,
                                         self.objects_list[obj_index].angularVelocity,)
            self.change_obj_parameters(obj_id, obj_index)
        # execute the control
        self.move_robot_JointPosition(joint_states)
        # collision check: add robot
        collision_detection_obj_id_.append(self.robot_id)
        # collision check: add board
        collision_detection_obj_id_.append(self.board_id_1)
        # collision check
        for obj_index in range(self.OBJECT_NUM):
            obj_id = self.objects_list[obj_index].no_visual_par_id
            # get linearVelocity and angularVelocity of the object from each particle
            linearVelocity, angularVelocity = self.p_env.getBaseVelocity(obj_id)
            obj_cur_pos, obj_cur_ori = self.get_item_pos(obj_id)
            normal_x = obj_cur_pos[0]
            normal_y = obj_cur_pos[1]
            normal_z = obj_cur_pos[2]
            pb_quat = obj_cur_ori
            # add noise on pose of each particle
            if self.OBJ_MOTION_MODEL_NOISE_FLAG == True:
                normal_x, normal_y, normal_z, pb_quat = self.add_noise_pose(obj_cur_pos, obj_cur_ori)
                self.p_env.resetBasePositionAndOrientation(obj_id, [normal_x, normal_y, normal_z], pb_quat)
                collision_detection_obj_id_.append(obj_id)
                obj_pose_3_1 = [normal_x, normal_y, normal_z, pb_quat]
                normal_x, normal_y, normal_z, pb_quat = self.collision_check(collision_detection_obj_id_,
                                                                             obj_cur_pos, obj_cur_ori,
                                                                             obj_id, obj_index, obj_pose_3_1)

            self.update_object_pose_PB(obj_index, normal_x, normal_y, normal_z, pb_quat, linearVelocity, angularVelocity)
        self.p_env.stepSimulation()
        return_results = self.get_objects_pose(par_index)
        return return_results


    def init_set_sim_robot_JointPosition(self, joint_states):
        num_joints = 9
        for joint_index in range(num_joints):
            if joint_index == 7 or joint_index == 8:
                self.p_env.resetJointState(self.robot_id,
                                           joint_index+2,
                                           targetValue=joint_states[joint_index])
            else:
                self.p_env.resetJointState(self.robot_id,
                                           joint_index,
                                           targetValue=joint_states[joint_index])

    def move_robot_JointPosition(self, joint_states):
        num_joints = 9
        for joint_index in range(num_joints):
            if joint_index == 7 or joint_index == 8:
                self.p_env.setJointMotorControl2(self.robot_id, joint_index+2,
                                                 self.p_env.POSITION_CONTROL,
                                                 targetPosition=joint_states[joint_index])
            else:
                self.p_env.setJointMotorControl2(self.robot_id, joint_index,
                                                 self.p_env.POSITION_CONTROL,
                                                 targetPosition=joint_states[joint_index])
        for time_index in range(int(self.PF_UPDATE_INTERVAL_IN_SIM)):
            self.p_env.stepSimulation()
        return [("done", True)]

    def compare_distance(self, par_index, pw_T_obj_obse_objects_pose_list, visual_by_DOPE_list, outlier_by_DOPE_list):
        weight =  1.0 / self.PARTICLE_NUM
        weights_list = [weight] * self.OBJECT_NUM
        for obj_index in range(self.OBJECT_NUM):
            self.objects_list[obj_index].w = weight
        # at least one object is detected by camera
        if (sum(visual_by_DOPE_list)<self.OBJECT_NUM) and (sum(outlier_by_DOPE_list)<self.OBJECT_NUM):
            for obj_index in range(self.OBJECT_NUM):
                obj_name = self.OBJECT_NAME_LIST[obj_index]
                weight =  1.0 / self.PARTICLE_NUM
                obj_visual = visual_by_DOPE_list[obj_index]
                obj_outlier = outlier_by_DOPE_list[obj_index]
                # obj_visual=0 means DOPE detects the object[obj_index]
                # obj_visual=1 means DOPE does not detect the object[obj_index] and skip this loop
                # obj_outlier=0 means DOPE detects the object[obj_index]
                # obj_outlier=1 means DOPE detects the object[obj_index], but we judge it is outlier and skip this loop
                if obj_visual==0 and obj_outlier==0:
                    obj_x = self.objects_list[obj_index].pos[0]
                    obj_y = self.objects_list[obj_index].pos[1]
                    obj_z = self.objects_list[obj_index].pos[2]
                    obj_ori = self.quaternion_correction(self.objects_list[obj_index].ori)
                    obse_obj_pos = pw_T_obj_obse_objects_pose_list[obj_index].pos
                    obse_obj_ori = pw_T_obj_obse_objects_pose_list[obj_index].ori # pybullet x,y,z,w
                    # make sure theta between -pi and pi
                    obse_obj_ori = self.quaternion_correction(obse_obj_ori)
                    mean = 0
                    # position weight
                    dis_x = abs(obj_x - obse_obj_pos[0])
                    dis_y = abs(obj_y - obse_obj_pos[1])
                    dis_z = abs(obj_z - obse_obj_pos[2])
                    dis_xyz = math.sqrt(dis_x ** 2 + dis_y ** 2 + dis_z ** 2)
                    weight_xyz = self.normal_distribution(dis_xyz, mean, self.OBJ_SIGMA_POS_FOR_OBS_WEIGHT_DICT[obj_name])
                    # rotation weight
                    obse_obj_quat = Quaternion(x=obse_obj_ori[0], y=obse_obj_ori[1], z=obse_obj_ori[2], w=obse_obj_ori[3]) # Quaternion(): w,x,y,z
                    par_quat = Quaternion(x=obj_ori[0], y=obj_ori[1], z=obj_ori[2], w=obj_ori[3])
                    err_bt_par_obse = par_quat * obse_obj_quat.inverse
                    err_bt_par_obse_corr = self.quaternion_correction([err_bt_par_obse.x, err_bt_par_obse.y, err_bt_par_obse.z, err_bt_par_obse.w])
                    err_bt_par_obse_corr_quat = Quaternion(x=err_bt_par_obse_corr[0], y=err_bt_par_obse_corr[1], z=err_bt_par_obse_corr[2], w=err_bt_par_obse_corr[3]) # Quaternion(): w,x,y,z
                    cos_theta_over_2 = err_bt_par_obse_corr_quat.w
                    sin_theta_over_2 = math.sqrt(err_bt_par_obse_corr_quat.x ** 2 + err_bt_par_obse_corr_quat.y ** 2 + err_bt_par_obse_corr_quat.z ** 2)
                    theta_over_2 = math.atan2(sin_theta_over_2, cos_theta_over_2)
                    theta = theta_over_2 * 2.0
                    weight_ang = self.normal_distribution(theta, mean, self.OBJ_SIGMA_ANG_FOR_OBS_WEIGHT_DICT[obj_name])
                    weight = weight_xyz * weight_ang
                    self.objects_list[obj_index].w = weight
                    weights_list[obj_index] = weight
                else:
                    self.objects_list[obj_index].w = weight
                    weights_list[obj_index] = weight

        return [(str(par_index), weights_list)]

    def generate_random_pose(self, pw_T_obj_obse_pos, pw_T_obj_obse_ori, noise_x, noise_y, noise_z, noise_ang):
        quat = pw_T_obj_obse_ori # x,y,z,w
        quat_QuatStyle = Quaternion(x=quat[0],y=quat[1],z=quat[2],w=quat[3]) # w,x,y,z
        x = self.add_noise_to_init_par(pw_T_obj_obse_pos[0], noise_x)
        y = self.add_noise_to_init_par(pw_T_obj_obse_pos[1], noise_y)
        z = self.add_noise_to_init_par(pw_T_obj_obse_pos[2], noise_z)
        random_dir = random.uniform(0, 2*math.pi)
        z_axis = random.uniform(-1,1)
        x_axis = math.cos(random_dir) * math.sqrt(1 - z_axis ** 2)
        y_axis = math.sin(random_dir) * math.sqrt(1 - z_axis ** 2)
        angle_noise = self.add_noise_to_init_par(0, noise_ang)
        w_quat = math.cos(angle_noise/2.0)
        x_quat = math.sin(angle_noise/2.0) * x_axis
        y_quat = math.sin(angle_noise/2.0) * y_axis
        z_quat = math.sin(angle_noise/2.0) * z_axis
        ###nois_quat(w,x,y,z); new_quat(w,x,y,z)
        nois_quat = Quaternion(x=x_quat, y=y_quat, z=z_quat, w=w_quat)
        new_quat = nois_quat * quat_QuatStyle
        ###pb_quat(x,y,z,w)
        pb_quat = [new_quat[1], new_quat[2], new_quat[3], new_quat[0]]
        return [x, y, z], pb_quat

    def set_particle_in_each_sim_env(self, single_particle):
        for obj_index in range(self.OBJECT_NUM):
            x = single_particle[obj_index].pos[0]
            y = single_particle[obj_index].pos[1]
            z = single_particle[obj_index].pos[2]
            pb_quat = single_particle[obj_index].ori
            linearVelocity = single_particle[obj_index].linearVelocity
            angularVelocity = single_particle[obj_index].angularVelocity
            self.update_object_pose_PB(obj_index, x, y, z, pb_quat, linearVelocity, angularVelocity)
        return [("done", True)]

    def update_object_pose_PB(self, obj_index, x, y, z, pb_quat, linearVelocity, angularVelocity):
        self.objects_list[obj_index].pos = [x, y, z]
        self.objects_list[obj_index].ori = pb_quat
        self.objects_list[obj_index].linearVelocity = linearVelocity
        self.objects_list[obj_index].angularVelocity = angularVelocity
        obj_id = self.objects_list[obj_index].no_visual_par_id
        self.p_env.resetBasePositionAndOrientation(obj_id, [x, y, z], pb_quat)

    def add_noise_to_init_par(self, current_pos, sigma_init):
        mean = current_pos
        sigma = sigma_init
        new_pos_is_added_noise = self.take_easy_gaussian_value(mean, sigma)
        return new_pos_is_added_noise
    
    def take_easy_gaussian_value(self, mean, sigma):
        normal = random.normalvariate(mean, sigma)
        return normal

    def get_item_pos(self, item_id):
        item_info = self.p_env.getBasePositionAndOrientation(item_id)
        return item_info[0], item_info[1]

    def getLinkStates(self):
        all_links_info = self.p_env.getLinkStates(self.robot_id, range(self.PANDA_ROBOT_LINK_NUMBER + 2), computeForwardKinematics=True) # 11+2; range: [0,13)
        all_links_info = all_links_info[:12]
        return [("links_info", all_links_info)]

    # add noise
    def add_noise_pose(self, obj_cur_pos, obj_cur_ori): # obj_cur_pos: x,y,z; obj_cur_ori: x,y,z,w
        # add noise to pos of object
        normal_x = self.add_noise_2_par(obj_cur_pos[0])
        normal_y = self.add_noise_2_par(obj_cur_pos[1])
        normal_z = self.add_noise_2_par(obj_cur_pos[2])
        # add noise to ang of object
        quat_QuatStyle = Quaternion(x=obj_cur_ori[0], y=obj_cur_ori[1], z=obj_cur_ori[2], w=obj_cur_ori[3])# w,x,y,z
        random_dir = random.uniform(0, 2*math.pi)
        z_axis = random.uniform(-1,1)
        x_axis = math.cos(random_dir) * math.sqrt(1 - z_axis ** 2)
        y_axis = math.sin(random_dir) * math.sqrt(1 - z_axis ** 2)
        angle_noise = self.add_noise_2_ang(0)
        w_quat = math.cos(angle_noise/2.0)
        x_quat = math.sin(angle_noise/2.0) * x_axis
        y_quat = math.sin(angle_noise/2.0) * y_axis
        z_quat = math.sin(angle_noise/2.0) * z_axis
        ###nois_quat(w,x,y,z); new_quat(w,x,y,z)
        nois_quat = Quaternion(x=x_quat, y=y_quat, z=z_quat, w=w_quat)
        new_quat = nois_quat * quat_QuatStyle
        ###pb_quat(x,y,z,w); pb_quat(x,y,z,w)
        pb_quat = [new_quat[1],new_quat[2],new_quat[3],new_quat[0]]
        new_angle = p.getEulerFromQuaternion(pb_quat)
        pb_quat = p.getQuaternionFromEuler(new_angle)
        # pipe.send()
        return normal_x, normal_y, normal_z, pb_quat

    def add_noise_2_par(self, current_pos):
        mean = current_pos
        sigma = self.OBJ_MOTION_MODEL_POS_NOISE
        new_pos_is_added_noise = self.take_easy_gaussian_value(mean, sigma)
        return new_pos_is_added_noise

    def add_noise_2_ang(self, cur_angle):
        mean = cur_angle
        sigma = self.OBJ_MOTION_MODEL_ANG_NOISE
        new_ang_is_added_noise = self.take_easy_gaussian_value(mean, sigma)
        return new_ang_is_added_noise
    
    # make sure all quaternions all between -pi and +pi
    def quaternion_correction(self, quaternion): # x,y,z,w
        new_quat = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]) # w,x,y,z
        cos_theta_over_2 = new_quat.w
        sin_theta_over_2 = math.sqrt(new_quat.x ** 2 + new_quat.y ** 2 + new_quat.z ** 2)
        theta_over_2 = math.atan2(sin_theta_over_2,cos_theta_over_2)
        theta = theta_over_2 * 2.0
        while theta >= math.pi:
            theta = theta - 2.0*math.pi
        while theta <= -math.pi:
            theta = theta + 2.0*math.pi
        new_quaternion = [math.sin(theta/2.0)*(new_quat.x/sin_theta_over_2), math.sin(theta/2.0)*(new_quat.y/sin_theta_over_2), math.sin(theta/2.0)*(new_quat.z/sin_theta_over_2), math.cos(theta/2.0)]
        return new_quaternion

    def normal_distribution(self, x, mean, sigma):
        return sigma * np.exp(-1*((x-mean)**2)/(2*(sigma**2)))/(math.sqrt(2*np.pi)* sigma)
    

    def collision_check(self, collision_detection_obj_id_, obj_cur_pos, obj_cur_ori, obj_id, obj_index, obj_pose_3_1):
        normal_x = obj_pose_3_1[0]
        normal_y = obj_pose_3_1[1]
        normal_z = obj_pose_3_1[2]
        pb_quat = obj_pose_3_1[3]
        nTries = 0
        collision_id_length = len(collision_detection_obj_id_)
        while nTries < 20:
            nTries = nTries + 1
            flag = 0
            for check_num in range(collision_id_length-1):
                self.p_env.stepSimulation()
                # will return all collision points
                contacts = self.p_env.getContactPoints(bodyA=collision_detection_obj_id_[check_num], # robot, other object...
                                                       bodyB=collision_detection_obj_id_[-1]) # main(target) object
                for contact in contacts:
                    contactNormalOnBtoA = contact[7]
                    contact_dis = contact[8]
                    # if contact_dis < -0.001: # means: positive for separation, negative for penetration
                    # if contact_dis < -0.02: # means: positive for separation, negative for penetration
                    if contact_dis < -0.0005: # means: positive for separation, negative for penetration
                        normal_x, normal_y, normal_z, pb_quat = self.add_noise_pose(obj_cur_pos, obj_cur_ori)
                        self.p_env.resetBasePositionAndOrientation(obj_id, [normal_x, normal_y, normal_z], pb_quat)
                        flag = 1
                        break
                if flag == 1:
                    break
            if flag == 0:
                break
        if nTries >= 20:
            print("WARNING: Could not find a non-colliding pose after motion noise. Moving particle object to noise-less pose.")
            self.p_env.resetBasePositionAndOrientation(obj_id, obj_cur_pos, obj_cur_ori)
        return normal_x, normal_y, normal_z, pb_quat


    # change particle parameters
    def change_obj_parameters(self, obj_id, obj_index):
        obj_name = self.OBJECT_NAME_LIST[obj_index]
        mass_a = self.take_easy_gaussian_value(self.OBJ_MASS_MEAN_DICT[obj_name], self.OBJ_MASS_SIGMA_DICT[obj_name])
        if mass_a < self.OBJ_MASS_MIN_MEAN_DICT[obj_name]:
            mass_a = self.OBJ_MASS_MIN_MEAN_DICT[obj_name]
        lateralFriction = self.take_easy_gaussian_value(self.OBJ_FRICTION_MEAN_DICT[obj_name], self.OBJ_FRICTION_SIGMA_DICT[obj_name])
        spinningFriction = self.take_easy_gaussian_value(self.OBJ_FRICTION_MEAN_DICT[obj_name], self.OBJ_FRICTION_SIGMA_DICT[obj_name])
        rollingFriction = self.take_easy_gaussian_value(self.OBJ_FRICTION_MEAN_DICT[obj_name], self.OBJ_FRICTION_SIGMA_DICT[obj_name])
        if lateralFriction < self.OBJ_FRICTION_MIN_MEAN_DICT[obj_name]:
            lateralFriction = self.OBJ_FRICTION_MIN_MEAN_DICT[obj_name]
        if spinningFriction < self.OBJ_FRICTION_MIN_MEAN_DICT[obj_name]:
            spinningFriction = self.OBJ_FRICTION_MIN_MEAN_DICT[obj_name]
        if rollingFriction < self.OBJ_FRICTION_MIN_MEAN_DICT[obj_name]:
            rollingFriction = self.OBJ_FRICTION_MIN_MEAN_DICT[obj_name]
        restitution = self.take_easy_gaussian_value(self.OBJ_RESTITUTION_MEAN, self.OBJ_RESTITUTION_SIGMA)
        self.p_env.changeDynamics(obj_id, -1, mass = mass_a, 
                                  lateralFriction = lateralFriction, 
                                  spinningFriction = spinningFriction, 
                                  rollingFriction = rollingFriction, 
                                  restitution = restitution)

        self.p_env.changeDynamics(self.table_id_1, 0, 
                                lateralFriction = lateralFriction, 
                                spinningFriction = spinningFriction, 
                                rollingFriction = self.OBJ_FRICTION_MIN_MEAN_DICT[obj_name], 
                                restitution = restitution)
        self.p_env.changeDynamics(self.robot_id, 10, 
                                lateralFriction = lateralFriction, 
                                spinningFriction = spinningFriction, 
                                rollingFriction = self.OBJ_FRICTION_MIN_MEAN_DICT[obj_name], 
                                restitution = restitution)
        self.p_env.changeDynamics(self.robot_id, 11, 
                                lateralFriction = lateralFriction, 
                                spinningFriction = spinningFriction, 
                                rollingFriction = self.OBJ_FRICTION_MIN_MEAN_DICT[obj_name], 
                                restitution = restitution)

    def getCameraInPybulletWorldPose44(self, tf_listener, pw_T_rob_sim_4_4):
        if self.OPTITRACK_FLAG == True and self.LOCATE_CAMERA_FLAG == "opti":
            realsense_tf = '/RealSense' # (use Optitrack)
        else:
            realsense_tf = '/ar_tracking_camera_frame' # (do not use Optitrack)
        if self.GAZEBO_FLAG == True:
            realsense_tf = '/realsense_camera'
        # mark
        while_time = 0
        while True:
            while_time = while_time + 1
            if while_time > 1000:
                # print("In launch_camera.py: Can not find the pose of the camera!!!! You need to wait a while or try to debug")
                pass
            try:
                (trans_camera, rot_camera) = tf_listener.lookupTransform('/panda_link0', realsense_tf, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        camRGB_T_camD_tf_pos = [0.015, 0.0, 0.0]
        camRGB_T_camD_tf_ori = [0.0, 0.0, -0.008, 1] # x, y, z, w

        camRGB_T_camD_tf_3_3 = np.array(p.getMatrixFromQuaternion(camRGB_T_camD_tf_ori)).reshape(3, 3)
        camRGB_T_camD_tf_3_4 = np.c_[camRGB_T_camD_tf_3_3, camRGB_T_camD_tf_pos]  # Add position to create 3x4 matrix
        camRGB_T_camD_tf_4_4 = np.r_[camRGB_T_camD_tf_3_4, [[0, 0, 0, 1]]]  # Convert to 4x4 homogeneous matrix

        rob_T_camRGB_tf_pos = list(trans_camera)
        rob_T_camRGB_tf_ori = list(rot_camera)
        rob_T_camRGB_tf_3_3 = np.array(p.getMatrixFromQuaternion(rob_T_camRGB_tf_ori)).reshape(3, 3)
        rob_T_camRGB_tf_3_4 = np.c_[rob_T_camRGB_tf_3_3, rob_T_camRGB_tf_pos]  # Add position to create 3x4 matrix
        rob_T_camRGB_tf_4_4 = np.r_[rob_T_camRGB_tf_3_4, [[0, 0, 0, 1]]]  # Convert to 4x4 homogeneous matrix

        rob_T_camD_tf_4_4 = np.dot(rob_T_camRGB_tf_4_4, camRGB_T_camD_tf_4_4)
        self.pw_T_camD_tf_4_4 = np.dot(pw_T_rob_sim_4_4, rob_T_camD_tf_4_4)
        self.compute_cam_pose_flag = 1

        return self.pw_T_camD_tf_4_4 

    #################################################################

    def create_new_physicsEnv_Z(self, pw_T_rob_sim_pose_list_alg):
        """
        get pose of the end-effector of the robot arm from joints of robot arm
        """
        self.p_track_fk_env = bc.BulletClient(connection_mode=p.DIRECT) # DIRECT, GUI_SERVER
        self.p_track_fk_env.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        real_robot_start_pos = pw_T_rob_sim_pose_list_alg[0].pos
        real_robot_start_ori = pw_T_rob_sim_pose_list_alg[0].ori

        self.track_fk_rob_id = self.p_track_fk_env.loadURDF(os.path.expanduser("~/project/data/bullet3-master/examples/pybullet/gym/pybullet_data/franka_panda/panda.urdf"),
                                                            real_robot_start_pos,
                                                            real_robot_start_ori,
                                                            useFixedBase=1)

    def move_rob_in_Z(self, joint_states):
        """
        move the joint state of the robot in the fake env world 
        """
        num_joints = 9
        for joint_index in range(num_joints):
            if joint_index == 7 or joint_index == 8:
                self.p_track_fk_env.resetJointState(self.track_fk_rob_id,
                                                    joint_index+2,
                                                    targetValue=joint_states[joint_index])
            else:
                self.p_track_fk_env.resetJointState(self.track_fk_rob_id,
                                                    joint_index,
                                                    targetValue=joint_states[joint_index])

    def get_rob_end_effector_state_in_Z(self):
        rob_link_9_pose = self.p_track_fk_env.getLinkState(self.track_fk_rob_id, 9)
        return rob_link_9_pose
    
    def get_all_rob_links_info_in_Z(self):
        all_links_info = self.p_track_fk_env.getLinkStates(self.track_fk_rob_id, range(self.PANDA_ROBOT_LINK_NUMBER + 2), computeForwardKinematics=True) # 11+2; range: [0,13)
        return all_links_info

    def get_rob_base_state_in_Z(self):
        base_link_info = self.p_track_fk_env.getBasePositionAndOrientation(self.track_fk_rob_id) # base (link0)
        return base_link_info
        









