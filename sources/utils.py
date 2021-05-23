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
            self._joint_names = [
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
            self._joint_names = [
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
        super().__init__(0, "Telo_Surrogat", joint_names=self._joint_names)
        self.imu = Dummy("Dummy_H")

        self._hand_collections = [sim.simGetCollectionHandle(name) for name in ["LeftHand", "RightHand"]]
        self._leg_collections = [sim.simGetCollectionHandle(name) for name in ["LeftLeg", "RightLeg"]]
        self._floor_collection = sim.simGetCollectionHandle("Floor")
        self._head_collection = sim.simGetCollectionHandle("Head")
        self._body_collection = sim.simGetCollectionHandle("Body")

        self._init_joint_positions = self.get_joint_positions()

        self._imu_quaternion_abs_limit = 1
        self._imu_velocity_abs_limit = 1
        self._joint_forces_abs_limit = 5

    def reset(self):
        self.set_joint_target_positions(self._init_joint_positions)
        self.set_joint_positions(self._init_joint_positions)

    def check_self_collision(self):
        result = sim.simCheckCollision(*self._hand_collections) == 1
        result |= sim.simCheckCollision(*self._leg_collections) == 1
        result |= sim.simCheckCollision(self._head_collection, self._body_collection) == 1
        for limb in (self._hand_collections + self._leg_collections):
            for main_part in [self._head_collection, self._body_collection]:
                result |= sim.simCheckCollision(limb, main_part) == 1

        for hand in self._hand_collections:
            for leg in self._leg_collections:
                result |= sim.simCheckCollision(hand, leg) == 1
        return result

    def check_floor_collision(self):
        elements = [self._head_collection, self._body_collection] + self._hand_collections
        result = False
        for element in elements:
            result |= sim.simCheckCollision(self._floor_collection, element) == 1

        return result

    def check_leg_floor_collision(self):
        result = False
        for leg in self._leg_collections:
            result |= sim.simCheckCollision(self._floor_collection, leg) == 1
        return result

    def get_imu_position(self):
        return self.imu.get_position()

    def get_imu_quaternion(self):
        return self.imu.get_quaternion()

    def get_imu_quaternion_limits(self):
        low = -np.ones(4) * self._imu_quaternion_abs_limit
        high = np.ones(4) * self._imu_quaternion_abs_limit
        return low, high

    def get_imu_velocity(self):
        base_velocity = np.hstack(self.imu.get_velocity())
        velocity = np.clip(base_velocity, -self._imu_velocity_abs_limit, self._imu_velocity_abs_limit)
        return velocity

    def get_joint_forces_limits(self):
        high = np.ones(len(self._joint_names)) * self._joint_forces_abs_limit
        low = -high
        return low, high

    def get_imu_velocity_limits(self):
        low = -np.ones(6) * self._imu_velocity_abs_limit
        high = np.ones(6) * self._imu_velocity_abs_limit
        return low, high

    def get_joint_positions(self):
        return np.array(super().get_joint_positions())

    def get_joint_positions_limits(self):
        cyclics, join_intervals = self.get_joint_intervals()
        low = []
        high = []

        for i, cyclic in enumerate(cyclics):
            low.append(join_intervals[i][0])
            high.append(join_intervals[i][0] + join_intervals[i][1])

        return np.array(low), np.array(high)

    def get_joint_velocities_limits(self):
        high = np.array(self.get_joint_upper_velocity_limits())
        low = -high
        return low, high


class VelocityAliveReward(object):
    def __init__(self, robot, dt, alpha=1.25, count_steps=50, max_step=600, goal_threshold=0.05):
        self._robot = robot
        self._start_position = robot.get_position()
        self._target_position = target_position
        self._dt = dt
        self._alpha = alpha
        self._count_steps = count_steps
        self._max_step = max_step
        self._goal_threshold = goal_threshold
        self._target_velocity = np.array([1, 0])
        self._action_space_dim = action_space_dim
        self._fall_lower_bound = 0
        self._fall_upper_bound = 40
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
        if self._robot.check_floor_collision():
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
        smooth_reward = -1.5 * ((self._prev_ctrl - self._current_ctrl) ** 2).sum()**.5 / np.pi

        info = dict(
            smooth_reward=smooth_reward
        )
        return smooth_reward, info

    def _get_collision_reward(self):
        collision_reward = 0
        if self._robot.check_self_collision():
            collision_reward = -1

        info = dict(
            collision_reward=collision_reward
        )
        return collision_reward, info

    def _done(self):
        fall_prob, _ = self._get_fall_prob()
        floor_collision = self._robot.check_leg_floor_collision()
        # print("floor_collision:", floor_collision)
        # return floor_collision
        if fall_prob > 0.6 or self._robot.check_self_collision() or not floor_collision:
            return True
        return False

    def _update_from_state(self, current_state):
        if self._current_ctrl is not None:
            self._prev_ctrl = self._current_ctrl
        self._current_ctrl = current_state[:self._action_space_dim]
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


class CombinedReward(object):
    def __init__(self, simulation):
        self._simulation = simulation
        self._fall_lower_bound = 0
        self._fall_upper_bound = 24
        self._target_velocity = np.array([1, 0])
        self._maximum_velocity_abs_limit = 2
        self._maximum_mean_joint_distance = 0.06
        self._minimum_mean_joint_distance = 0.01
        self._single_joint_max_abs_force = 1.1
        self._single_joint_min_abs_force = 0.3
        self._fall_reward_alpha = 1
        self._velocity_reward_alpha = 1
        self._smooth_reward_alpha = 1
        self._force_reward_alpha = 1
        self._smooth_reward_iter_count = 30

        self._imu_positions_queue = None
        self._joint_positions_limits = None
        self._prev_joint_positions = None
        self._curr_joint_positions = None
        self._set_up_inner_states()

    def _set_up_inner_states(self):
        self._imu_positions_queue = [self._simulation.robot.get_imu_position(), ]
        self._joint_positions_limits = self._simulation.robot.get_joint_positions_limits()

        self._prev_joint_positions = self._simulation.robot.get_joint_positions()
        self._curr_joint_positions = self._prev_joint_positions

    def update_inner_states(self):
        self._prev_joint_positions = self._curr_joint_positions
        self._curr_joint_positions = self._simulation.robot.get_joint_positions()

        self._imu_positions_queue.append(
            self._simulation.robot.get_imu_position()
        )
        if len(self._imu_positions_queue) > self._smooth_reward_iter_count:
            self._imu_positions_queue.pop(0)

    def _get_fall_prob(self):
        zyx = self._quat_to_euler(self._simulation.robot.get_imu_quaternion())
        pos = self._simulation.robot.get_imu_position()

        yx = zyx[1:]
        zpos = pos[2]
        fall_metric = (yx ** 2).sum()**.5
        fall_prob = max(min(fall_metric, self._fall_upper_bound), self._fall_lower_bound) - self._fall_lower_bound
        fall_prob /= self._fall_upper_bound - self._fall_lower_bound
        info = dict(
            fall_metric=fall_metric,
            y=yx[0],
            x=yx[1],
            zpos=zpos,
            floor_collision=False
        )

        if zpos < 0.3:
            fall_prob = 1

        if self._simulation.robot.check_floor_collision():
            fall_prob = 1
            info.update(
                floor_collision=True
            )

        info.update(
            fall_prob=fall_prob
        )
        return fall_prob, info

    def _done(self):
        fall_prob, _ = self._get_fall_prob()
        floor_collision = self._simulation.robot.check_leg_floor_collision()
        self_collision = self._simulation.robot.check_self_collision()

        info = dict(
            fall_prob=fall_prob,
            floor_collision=floor_collision,
            self_collision=self_collision,
        )
        if fall_prob > 0.99 or self_collision or not floor_collision:
            return True, info
        return False, info

    def _get_fall_reward(self):
        fall_prob, info = self._get_fall_prob()

        fall_reward = 1 - fall_prob

        info.update(
            fall_reward=fall_reward,
            fall_prob=fall_prob
        )
        return fall_reward, info

    def _get_velocity_reward(self):
        # TODO: try to replace with imu velocity
        current_velocity = (self._imu_positions_queue[-1] - self._imu_positions_queue[0])[:2] / (self._simulation.dt * len(self._imu_positions_queue))
        current_velocity_abs = (current_velocity**2).sum() ** .5
        target_velocity_abs = min(current_velocity_abs, self._maximum_velocity_abs_limit)
        current_velocity *= target_velocity_abs / current_velocity_abs

        velocity_cross_prod = current_velocity @ self._target_velocity
        velocity_reward = velocity_cross_prod / self._maximum_velocity_abs_limit

        info = dict(
            current_velocity=current_velocity,
            velocity_cross_prod=velocity_cross_prod,
            velocity_reward=velocity_reward
        )
        return velocity_reward, info

    def _get_smooth_reward(self):
        joint_distance = np.abs(self._prev_joint_positions - self._curr_joint_positions)
        normalized_joint_distance = joint_distance / (self._joint_positions_limits[1] - self._joint_positions_limits[0])
        mean_joint_distance = np.mean(normalized_joint_distance)
        mean_joint_distance_clipped = max(self._minimum_mean_joint_distance, min(float(mean_joint_distance), self._maximum_mean_joint_distance))
        mean_joint_distance_normalized = (mean_joint_distance_clipped - self._minimum_mean_joint_distance) / (self._maximum_mean_joint_distance - self._minimum_mean_joint_distance)

        smooth_reward = 1 - mean_joint_distance_normalized

        info = dict(
            mean_joint_distance=mean_joint_distance,
            mean_joint_distance_normalized=mean_joint_distance_normalized,
            smooth_reward=smooth_reward
        )
        return smooth_reward, info

    def _get_force_reward(self):
        forces = self._simulation.robot.get_joint_forces()
        sum_forces = np.abs(forces).sum()
        min_sum_forces = self._single_joint_min_abs_force * len(forces)
        max_sum_forces = self._single_joint_max_abs_force * len(forces)
        sum_forces_normalized = (max(min_sum_forces, min(sum_forces, max_sum_forces)) - min_sum_forces) / (max_sum_forces - min_sum_forces)

        force_reward = 1 - sum_forces_normalized
        info = dict(
            sum_forces=sum_forces,
            sum_forces_normalized=sum_forces_normalized,
            force_reward=force_reward
        )
        return force_reward, info

    def step(self):
        """
        It's essential to call this after simulation step
        :return:
        r - reward
        done - if need to finish simulation
        info - dict with debug information
        """
        self.update_inner_states()

        final_info = dict()
        fall_reward, info = self._get_fall_reward()
        final_info.update(info)
        velocity_reward, info = self._get_velocity_reward()
        final_info.update(info)
        smooth_reward, info = self._get_smooth_reward()
        final_info.update(info)
        force_reward, info = self._get_force_reward()
        final_info.update(info)

        rewards = np.array([fall_reward, velocity_reward, smooth_reward, force_reward])
        alphas = np.array([self._fall_reward_alpha, self._velocity_reward_alpha, self._smooth_reward_alpha, self._force_reward_alpha])

        r = rewards @ alphas / alphas.sum()
        done, info = self._done()
        final_info.update(info)
        final_info.update(
            reward=r
        )
        return r, done, final_info

    def reset(self):
        self._set_up_inner_states()

    @staticmethod
    def _quat_to_euler(quat):
        r = R.from_quat(quat)
        return r.as_euler('zyx', degrees=True)


class Simulation(object):
    def __init__(
        self,
        scene: str,
        dt: float,
        headless_mode: bool,
        foot_only_mode: bool,
    ):
        self.dt = dt
        self._pr = PyRep()
        self._pr.launch(scene, headless=headless_mode)
        if not headless_mode:
            self._clear_gui()

        self._pr.set_simulation_timestep(dt)
        self._pr.start()

        self.robot = StarkitRobot(foot_only_mode=foot_only_mode)
        self.goal = self._create_goal()

        self._initial_robot_configuration = self.robot.get_configuration_tree()

    @staticmethod
    def _clear_gui():
        sim.simSetBoolParameter(simConst.sim_boolparam_browser_visible, False)
        sim.simSetBoolParameter(simConst.sim_boolparam_hierarchy_visible, False)
        sim.simSetBoolParameter(simConst.sim_boolparam_console_visible, False)

    def _create_goal(self):
        goal = Dummy.create(size=0.1)
        goal.set_renderable(True)
        goal.set_name('Goal')
        goal.set_position(self.robot.get_position() * np.array([-1, 1, 1]))
        return goal

    def get_state(self):
        joint_positions = self.robot.get_joint_positions()
        joint_forces = self.robot.get_joint_forces()
        joint_velocities = self.robot.get_joint_velocities()
        velocity = self.robot.get_imu_velocity()
        imu_quat = self.robot.get_imu_quaternion()

        state = np.hstack([joint_positions, joint_forces, joint_velocities, velocity, imu_quat])
        return state

    def get_state_limits(self):
        joint_positions_limits = self.robot.get_joint_positions_limits()
        joint_forces_limits = self.robot.get_joint_forces_limits()
        joint_velocities_limits = self.robot.get_joint_velocities_limits()
        velocity_limits = self.robot.get_imu_velocity_limits()
        imu_quat_limits = self.robot.get_imu_quaternion_limits()

        limits = list(zip(joint_positions_limits, joint_forces_limits, joint_velocities_limits, velocity_limits, imu_quat_limits))
        limits[0] = np.hstack(limits[0])
        limits[1] = np.hstack(limits[1])

        return limits

    def apply_action(self, action):
        self.robot.set_joint_target_positions(action)

    def get_action_limits(self):
        return self.robot.get_joint_positions_limits()

    def close(self):
        self._pr.stop()
        self._pr.shutdown()

    def step(self):
        self._pr.step()

    def reset(self):
        self._pr.set_configuration_tree(self._initial_robot_configuration)
        self.robot.reset()
        self._pr.step()


class SimulationEnvironment(gym.Env):
    def __init__(
        self,
        scene: str,
        dt: float = 0.02,
        headless_mode: bool = False,
        foot_only_mode: bool = False,
    ):
        self.simulation = Simulation(scene, dt, headless_mode, foot_only_mode)
        self.reward = CombinedReward(self.simulation)
        self._init_spaces()

    def _init_spaces(self):
        action_intervals = self.simulation.get_action_limits()
        state_intervals = self.simulation.get_state_limits()
        self.action_space = spaces.Box(low=action_intervals[0], high=action_intervals[1], dtype=np.float32)
        self.observation_space = spaces.Box(low=state_intervals[0], high=state_intervals[1], dtype=np.float32)

    def close(self):
        self.simulation.close()

    def step(self, action):
        self.simulation.apply_action(action)
        self.simulation.step()
        # TODO split reward state into several stages
        r, done, info = self.reward.step()

        state = self.simulation.get_state()
        info.update(
            action=action,
            state=state
        )
        return state, r, done, info

    def reset(self):
        self.simulation.reset()
        self.reward.reset()
        state = self.simulation.get_state()
        return state

    def seed(self, seed: int = None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode: str = 'human'):
        print('Not implemented yet')


class SurrogatPyRepEnvironment(gym.Env):
    def __init__(
         self,
         scene: str,
         dt: float = 0.02,
         headless_mode: bool = False,
         foot_only_mode: bool = False,
    ):
        # self.seed()
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
        self._reward_calc = VelocityAliveReward(self.action_space.shape[0], self._robot_position(), self._goal.get_position(), dt=dt, count_steps=30)

    def _init_spaces(self):
        cyclics, join_intervals = self._robot.get_joint_intervals()
        for i, cyclic in enumerate(cyclics):
            if cyclic:
                join_intervals[i] = [-np.pi, np.pi]
            else:
                join_intervals[i] = [join_intervals[i][0], join_intervals[i][0] + join_intervals[i][1]]
        action_intervals = np.array(join_intervals)
        self.action_space = spaces.Box(low=action_intervals.T[0], high=action_intervals.T[1], dtype=np.float32)

        quaternion_limits = np.ones((4, 2)) * np.array([-1, 1])
        velocity_limits = np.ones((6, 2)) * np.array([-1, 1]) * 2
        joint_forces_limits = np.ones(action_intervals.shape) * np.array([-1, 1]) * 5

        max_joint_velocities = np.array(self._robot.get_joint_upper_velocity_limits())[:, None]
        joint_velocities_limits = np.ones(action_intervals.shape) * np.array([-1, 1]) * max_joint_velocities

        state_intervals = np.vstack([action_intervals, joint_forces_limits, joint_velocities_limits, velocity_limits, quaternion_limits])
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
        joint_forces = self._robot.get_joint_forces()
        joint_velocities = self._robot.get_joint_velocities()
        imu_quat = self._robot_quaternion()
        velocity = self._robot_velocity()
        state = np.hstack([joint_positions, joint_forces, joint_velocities, velocity, imu_quat])
        return state

    def replay_step(self, state):
        joint_positions = state[:-4]
        self._robot.set_joint_target_positions(joint_positions)
        self._robot.set_joint_positions(joint_positions)
        self._pr.step()

        state = self._get_state()
        r, done, info = self._reward_calc.compute_reward(self._robot_position(), state, self._robot)
        info.update(
            action=joint_positions,
            state=state
        )
        return state, r, done, info

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
        self._robot.reset()
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
