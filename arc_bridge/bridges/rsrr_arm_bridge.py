import numpy as np
import mujoco

from .lcm2mujuco_bridge import Lcm2MujocoBridge
from arc_bridge.utils import *

class RsrrArmBridge(Lcm2MujocoBridge):
    def __init__(self, mj_model, mj_data, config):
        super().__init__(mj_model, mj_data, config)


    def parse_robot_specific_low_state(self):
        self.low_state.ft_sensor[:] = self.mj_data.sensordata[24:30]