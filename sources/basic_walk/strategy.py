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

        self.motion.sim_backend.start()

        while self.motion.act():
            pass

        self.motion.sim_backend.stop()
        self.motion.sim_backend.disable()


if __name__ == "__main__":
    player = Player()
    player.simulation()
