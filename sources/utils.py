from pyrep import PyRep
from pyrep.objects import Dummy
from pyrep.robots.robot_component import RobotComponent
from pyrep.backend import sim, simConst

import gym
from gym import spaces
from gym.utils import seeding

from scipy.spatial.transform import Rotation as R
import numpy as np


class StarkitRobot(RobotComponent):
    def __init__(self, foot_only_mode=False):
        if foot_only_mode:
            self._join_names = [
                'Leg_right_10',
                'Leg_right_9',
                'Leg_right_8',
                'Leg_right_7',
                'Leg_right_6',
                'Leg_right_5',
                'Tors1',
                'Leg_left_10',
                'Leg_left_9',
                'Leg_left_8',
                'Leg_left_7',
                'Leg_left_6',
                'Leg_left_5',
            ]
        else:
            self._join_names = [
                'Leg_right_10',
                'Leg_right_9',
                'Leg_right_8',
                'Leg_right_7',
                'Leg_right_6',
                'Leg_right_5',
                'hand_right_4',
                'hand_right_3',
                'hand_right_2',
                'hand_right_1',
                'Tors1',
                'Leg_left_10',
                'Leg_left_9',
                'Leg_left_8',
                'Leg_left_7',
                'Leg_left_6',
                'Leg_left_5',
                'hand_left_4',
                'hand_left_3',
                'hand_left_2',
                'hand_left_1'
            ]
        super().__init__(0, "Telo_Surrogat", joint_names=self._join_names)
        self.imu = Dummy("Dummy_H")
        self._collision_collection = sim.simGetCollectionHandle('Telo_Surrogat')
        self.initial_configuration = self.get_configuration_tree()

    def check_collision(self):
        handle = sim.sim_handle_all
        result, pairHandles = sim.simCheckCollision(self._collision_collection, handle) == 1
        return sim.simCheckCollision(self._collision_collection, handle) == 1


class VelocityAliveReward(object):
    def __init__(self, start_position, target_position, dt, alpha=1.25, count_steps=50, max_step=600, goal_threshold=0.05):
        self._start_position = start_position
        self._target_position = target_position
        self._dt = dt
        self._alpha = alpha
        self._count_steps = count_steps
        self._max_step = max_step
        self._goal_threshold = goal_threshold
        self._target_velocity = np.array([1, 0])

        self._fall_lower_bound = 0
        self._fall_upper_bound = 30
        self._positions_queue = [self._start_position, ]
        self._current_quat = None
        self._current_ctrl = None
        self._prev_ctrl = None
        self._robot = None

    def _quat_to_euler(self, quat):
        r = R.from_quat(quat)
        return r.as_euler('zyx', degrees=True)

    def _get_fall_prob(self):
        zyx = self._quat_to_euler(self._current_quat)
        pos = self._positions_queue[-1]

        yx = zyx[1:]
        zpos = pos[2]
        fall_metric = (yx ** 2).sum()**.5
        fall_prob = max(min(fall_metric, self._fall_upper_bound), self._fall_lower_bound) - self._fall_lower_bound
        fall_prob /= self._fall_upper_bound - self._fall_lower_bound
        if zpos < 0.3:
            fall_prob = 1

        info = dict(
            fall_metric=fall_metric,
            y=zyx[1],
            x=zyx[2],
            zpos=pos[2],
        )
        return fall_prob, info

    def _get_fall_reward(self):
        fall_prob, info = self._get_fall_prob()
        fall_reward = 0.3
        if fall_prob > 0.3:
            fall_reward = (0.3 - fall_prob) / 7 * 10

        info.update(
            fall_reward=fall_reward,
            fall_prob=fall_prob
        )
        return fall_reward, info

    def _get_velocity_reward(self):
        current_velocity = (self._positions_queue[-1] - self._positions_queue[0])[:2] / (self._dt * len(self._positions_queue))
        velocity_reward = current_velocity @ self._target_velocity
        velocity_reward = self._alpha * velocity_reward

        info = dict(
            velocity_reward=velocity_reward
        )
        return velocity_reward, info

    def _get_smooth_reward(self):
        smooth_reward = -((self._prev_ctrl - self._current_ctrl) ** 2).sum()**.5 / 2 / np.pi

        info = dict(
            smooth_reward=smooth_reward
        )
        return smooth_reward, info

    def _get_collision_reward(self):
        collision_reward = 0
        if self._robot.check_collision():
            collision_reward = -1

        info = dict(
            collision_reward=collision_reward
        )
        return collision_reward, info

    def _done(self):
        fall_prob, _ = self._get_fall_prob()
        if fall_prob > 0.6:
            return True
        return False

    def _update_from_state(self, current_state):
        if self._current_ctrl is not None:
            self._prev_ctrl = self._current_ctrl
        self._current_ctrl = current_state[:-4]
        self._current_quat = current_state[-4:]

    def reset(self, current_state):
        self._positions_queue = [self._start_position, ]
        self._update_from_state(current_state)

    def compute_reward(self, current_position, current_state, robot):
        self._positions_queue.append(current_position)
        self._robot = robot
        if len(self._positions_queue) > self._count_steps:
            self._positions_queue.pop(0)

        self._update_from_state(current_state)

        final_info = dict()
        fall_reward, info = self._get_fall_reward()
        final_info.update(info)
        velocity_reward, info = self._get_velocity_reward()
        final_info.update(info)
        smooth_reward, info = self._get_smooth_reward()
        final_info.update(info)
        collision_reward, info = self._get_collision_reward()
        final_info.update(info)

        r = fall_reward + velocity_reward + smooth_reward + collision_reward
        done = self._done()

        final_info.update(
            reward=r,
            done=done,
        )
        return r, done, final_info


class SurrogatPyRepEnvironment(gym.Env):
    def __init__(
         self,
         scene: str,
         dt: float = 0.02,
         headless_mode: bool = False,
         foot_only_mode: bool = False,
    ):
        self.seed()
        self._headless_mode = headless_mode
        self._pr = PyRep()
        self._pr.launch(scene, headless=headless_mode)
        if not headless_mode:
            self._clear_gui()

        self._pr.set_simulation_timestep(dt)
        self._pr.start()
        self._robot = StarkitRobot(foot_only_mode=foot_only_mode)
        self._create_goal()
        self._init_spaces()
#         self._reward_calc = DistanceFallReward(self._robot_position(), self._goal.get_position())
        self._reward_calc = VelocityAliveReward(self._robot_position(), self._goal.get_position(), dt=dt, count_steps=30)

    def _init_spaces(self):
        cyclics, join_intervals = self._robot.get_joint_intervals()
        for i, cyclic in enumerate(cyclics):
            if cyclic:
                join_intervals[i] = [-np.pi, np.pi]
        action_intervals = np.array(join_intervals)
        self.action_space = spaces.Box(low=action_intervals.T[0], high=action_intervals.T[1], dtype=np.float32)

        quaternion_limits = np.ones((4, 2)) * np.array([-1, 1])
        state_intervals = np.vstack([action_intervals, quaternion_limits])
        self.observation_space = spaces.Box(low=state_intervals.T[0], high=state_intervals.T[1], dtype=np.float32)

    def _robot_position(self):
        return self._robot.imu.get_position()

    def _robot_quaternion(self):
        return self._robot.imu.get_quaternion()

    def _create_goal(self):
        self._goal = Dummy.create(size=0.1)
        self._goal.set_renderable(True)
        self._goal.set_name('Goal')
        self._goal.set_position(self._robot_position() * np.array([-1, 1, 1]))

    @staticmethod
    def _clear_gui():
        sim.simSetBoolParameter(simConst.sim_boolparam_browser_visible, False)
        sim.simSetBoolParameter(simConst.sim_boolparam_hierarchy_visible, False)
        sim.simSetBoolParameter(simConst.sim_boolparam_console_visible, False)

    def _get_state(self):
        joint_positions = self._robot.get_joint_positions()
        imu_quat = self._robot_quaternion()
        state = np.hstack([joint_positions, imu_quat])
        return state

    def step(self, action):
        self._robot.set_joint_target_positions(action)
        self._pr.step()
        state = self._get_state()
        r, done, info = self._reward_calc.compute_reward(self._robot_position(), state, self._robot)
        info.update(
            action=action,
            state=state
        )
        return state, r, done, info

    def reset(self):
        self._pr.set_configuration_tree(self._robot.initial_configuration)
        self._pr.step()
        state = self._get_state()
        self._reward_calc.reset(state)
        return state

    def close(self):
        self._pr.stop()
        self._pr.shutdown()

    def seed(self, seed: int = None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode: str = 'human'):
        print('Not implemented yet')
