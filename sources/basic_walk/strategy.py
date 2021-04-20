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


from motion_sim import Glob, MotionSim


class Player(object):
    def __init__(self):
        self.glob = Glob(current_work_directory)
        self.motion = None

    def simulation(self):
        self.motion = MotionSim(self.glob)
        self.motion.sim_start()
        
        self.balancing_test_main_cycle()

        self.motion.sim_stop()
        self.motion.sim_disable()

    def balancing_test_main_cycle(self):
        self.motion.refresh_orientation()
        self.motion.amplitude = 20
        self.motion.fr1 = 0
        self.motion.fr2 = 24
        self.motion.initPoses = self.motion.fr2

        number_of_cycles = 5
        step_length = 64
        side_length = 0

        if self.motion.first_leg_is_right:
            invert = -1
        else:
            invert = 1

        self.motion.walk_initial_pose()
        number_of_cycles += 1
        for cycle in range(number_of_cycles):
            step_length_1 = step_length
            second_step_length = step_length_1
            if cycle == 0:
                step_length_1 = step_length/3
                second_step_length = step_length/3
            if cycle == 1:
                step_length_1 = step_length/3 * 2
                second_step_length = step_length/3 * 2

            self.motion.refresh_orientation()

            rotation = invert * self.motion.imu_body_yaw() * 1.2
            rotation = self.motion.normalize_rotation(rotation)
            self.motion.walk_cycle(
                step_length_1,
                side_length,
                rotation,
                cycle,
                number_of_cycles,
                second_step_length=second_step_length
            )


if __name__ == "__main__":
    player = Player()
    player.simulation()
