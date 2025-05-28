#!/usr/bin/python3

from abc import ABC, abstractmethod

class PhysicsEnv(ABC):
    @abstractmethod
    def init_env():
        pass