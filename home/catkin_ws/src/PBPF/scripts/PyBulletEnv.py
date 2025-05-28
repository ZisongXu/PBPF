#!/usr/bin/python3

# pybullet
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client as bc

class PhysicsEnv(ABC):
    @abstractmethod
    def init_env():
        pass