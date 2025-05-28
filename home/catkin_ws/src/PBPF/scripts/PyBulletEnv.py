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
    