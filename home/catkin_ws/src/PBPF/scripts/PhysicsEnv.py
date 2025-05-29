#!/usr/bin/python3

from abc import ABC, abstractmethod

class PhysicsEnv(ABC):

    @abstractmethod
    def init_env():
        pass

    @abstractmethod
    def add_robot():
        pass

    @abstractmethod
    def add_static_obstacles():
        pass

    @abstractmethod
    def add_target_objects():
        pass

    @abstractmethod
    def get_objects_pose():
        pass

    @abstractmethod
    def isAnyParticleInContact():
        pass

    @abstractmethod
    def motion_model():
        pass

    @abstractmethod
    def init_set_sim_robot_JointPosition():
        pass

    @abstractmethod
    def move_robot_JointPosition():
        pass

    @abstractmethod
    def update_object_pose_PB():
        pass

    @abstractmethod
    def get_item_pos():
        pass

    @abstractmethod
    def getLinkStates():
        pass

    @abstractmethod
    def collision_check():
        pass

    @abstractmethod
    def change_obj_parameters():
        pass

    @abstractmethod
    def create_new_physicsEnv_Z():
        pass

    @abstractmethod
    def move_rob_in_Z():
        pass

    @abstractmethod
    def get_rob_end_effector_state_in_Z():
        pass

    @abstractmethod
    def get_all_rob_links_info_in_Z():
        pass
        
    @abstractmethod
    def get_rob_base_state_in_Z():
        pass
















