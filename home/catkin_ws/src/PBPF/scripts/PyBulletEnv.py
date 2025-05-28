#!/usr/bin/python3

# pybullet
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client as bc
from PhysicsEnv import PhysicsEnv

class PyBulletEnv(PhysicsEnv):
    def __init__(self, parameter_info, **kwargs):
        
        # ============================================================================

        self.parameter_info = parameter_info
        
        # ============================================================================
        
        self.PHYSICS_SIMULATION = self.parameter_info['physics_simulation']
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

    def init_env(self):
        if self.SHOW_RAY == True:
            self.p_env = bc.BulletClient(connection_mode=p.GUI_SERVER) # DIRECT,GUI_SERVER
        else:
            self.p_env = bc.BulletClient(connection_mode=p.DIRECT) # DIRECT,GUI_SERVER
        if self.update_style_flag == "time":
            self.p_env.setTimeStep(self.sim_time_step)
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
        if self.task_flag == "1":
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

            if self.task_flag != "4": # slope
            board_pos_1 = [0.274, 0.581, 0.87575]
            board_ori_1 = self.p_env.getQuaternionFromEuler([math.pi/2,math.pi/2,0])
            self.board_id_1 = self.p_env.loadURDF(os.path.expanduser("~/project/object/others/board.urdf"), board_pos_1, board_ori_1, useFixedBase = 1)
            self.collision_detection_obj_id_collection.append(self.board_id_1)

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
        for obj_index in range(self.object_num):
            obj_obse_pos = self.pw_T_obj_obse_obj_list_alg[obj_index].pos
            obj_obse_ori = self.pw_T_obj_obse_obj_list_alg[obj_index].ori
            obj_obse_name = self.pw_T_obj_obse_obj_list_alg[obj_index].obj_name
            if print_object_name_flag == obj_index:
                print_object_name_flag = print_object_name_flag + 1
            particle_pos, particle_ori = self.generate_random_pose(obj_obse_pos, obj_obse_ori, 
                                                                   self.sigma_obs_x_for_init, self.sigma_obs_y_for_init, self.sigma_obs_z_for_init,
                                                                   self.sigma_obs_ang_for_init)
            gazebo_contain = ""
            if self.gazebo_flag == True:
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
            objPose = Particle(obj_obse_name, 0, particle_no_visual_id, particle_pos, particle_ori, 1/self.particle_num, 0, 0, 0)
            self.objects_list[obj_index] = objPose