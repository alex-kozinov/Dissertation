

import sys
import os
import math
import json
import time


current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
current_work_directory += '/'

sys.path.append(current_work_directory + 'Soccer/')
sys.path.append(current_work_directory + 'Soccer/Motion/')

from class_Motion import Glob
from class_Motion import *
from class_Motion_sim import*
from class_Motion_sim import MotionSim


class Player(object):
    def __init__(self):
        self.glob = Glob(current_work_directory)
        self.motion = None

    def simulation(self):
        self.motion = MotionSim(self.glob)
        self.motion.sim_start()
        self.common_init()

        self.balancing_test_main_cycle()

        self.motion.sim_Stop()
        self.motion.sim_Disable()

    def common_init(self):
        self.motion.activation()
        self.motion.falling_Flag = 0

    def balancing_test_main_cycle(self):
        self.motion.amplitude = 20
        self.motion.fr1 = 0
        self.motion.fr2 = 24
        self.motion.initPoses = self.motion.fr2

        number_Of_Cycles = 5
        stepLength = 64
        sideLength = 0

        if self.motion.first_Leg_Is_Right_Leg:
            invert = -1
        else:
            invert = 1

        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            secondStepLength = stepLength1
            if cycle == 0 :
                stepLength1 = stepLength/3
                secondStepLength = stepLength/3
            if cycle == 1 :
                stepLength1 = stepLength/3 * 2
                secondStepLength = stepLength/3 * 2

            self.motion.refresh_Orientation()

            rotation = 0 + invert * self.motion.imu_body_yaw() * 1.2
            rotation = self.motion.normalize_rotation(rotation)
            self.motion.walk_cycle(
                stepLength1,
                sideLength,
                rotation,
                cycle,
                number_Of_Cycles,
                secondStepLength=secondStepLength
            )


if __name__=="__main__":
    player = Player()
    player.simulation()





