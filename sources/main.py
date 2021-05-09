from utils import SurrogatPyRepEnvironment
from basic_walk.utils import BaseAgent
import sys
import os
import time


import pickle
import matplotlib.pyplot as plt
import numpy as np


def main():
    with SurrogatPyRepEnvironment('scenes/basic_scene.ttt', headless_mode=True, foot_only_mode=True) as env:
        for i in range(1):
            agent = BaseAgent(random_mode=False, foot_only_mode=True)

            state = env.reset()
            for _ in range(300):
                action = agent.act(state)
                state, r, done, info = env.step(action)
                if done:
                    break


if __name__ == "__main__":
    main()
