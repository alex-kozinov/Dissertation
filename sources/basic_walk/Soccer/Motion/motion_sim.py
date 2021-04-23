#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
current_work_directory += '/'

with open("simulator_lib_directory.txt", "r") as f:
    simulator_lib_directory = f.read()
simulator_lib_directory = simulator_lib_directory.replace('\\', '/')
sys.path.append(simulator_lib_directory)


sys.path.append(current_work_directory + 'Soccer/')
sys.path.append(current_work_directory + 'Soccer/Motion/')

import sim
from compute_Alpha_v3 import Alpha


class Glob:
    def __init__(self, current_work_directory):
        self.current_work_directory = current_work_directory
        with open(current_work_directory + "Soccer/Init_params/Sim/Sim_params.json", "r") as f:
            self.params = json.loads(f.read())


class SimBackend(object):
    def __init__(self, joint_names, imu_name):
        self.joint_names = joint_names
        self.imu_name = imu_name
        self.joint_handles = []
        self.client_id = -1
        self.sim_step_counter = 0
        self.imu_handle = None

    def start(self):
        sim.simxFinish(-1)  # just in case, close all opened connections
        sim_thread_cycle_in_ms = 5
        self.client_id = sim.simxStart('127.0.0.1', -19997, True, True, 5000, sim_thread_cycle_in_ms)

        _, self.imu_handle = sim.simxGetObjectHandle(self.client_id, self.imu_name, sim.simx_opmode_blocking)
        for i, join_name in enumerate(self.joint_names):
            _, handle = sim.simxGetObjectHandle(self.client_id, join_name, sim.simx_opmode_blocking)
            self.joint_handles.append(handle)

        sim.simxGetIntegerParameter(self.client_id, sim.sim_intparam_program_version, sim.simx_opmode_streaming)
        time.sleep(0.1)
        sim.simxStartSimulation(self.client_id, sim.simx_opmode_oneshot)

    def wait_step(self):
        while True:
            sim.simxGetIntegerParameter(self.client_id, sim.sim_intparam_program_version, sim.simx_opmode_buffer)
            tim = sim.simxGetLastCmdTime(self.client_id)

            if tim > self.sim_step_counter:
                self.sim_step_counter = tim
                break
            time.sleep(0.001)

    def stop(self):
        sim.simxStopSimulation(self.client_id, sim.simx_opmode_oneshot)

    def disable(self):
        time.sleep(0.2)
        sim.simxFinish(self.client_id)

    def set_joint_positions(self, positions):
        sim.simxPauseCommunication(self.client_id, True)
        for position, joint_handle in zip(positions, self.joint_handles):
            sim.simxSetJointTargetPosition(
                self.client_id,
                joint_handle,
                position,
                sim.simx_opmode_oneshot
            )
        sim.simxPauseCommunication(self.client_id, False)

    def get_joint_positions(self):
        positions = []
        for i, joint_handle in enumerate(self.joint_handles):
            _, position = sim.simxGetJointPosition(self.client_id, joint_handle, sim.simx_opmode_blocking)
            positions.append(position)

        return positions

    def get_imu_quaternion(self):
        _, dummy_h_quaternion = sim.simxGetObjectQuaternion(self.client_id, self.imu_handle, -1, sim.simx_opmode_buffer)
        return dummy_h_quaternion


class Environment(object):
    def __init__(self):
        self.client_id = -1

    def __del__(self):
        pass

    def reset(self):
        pass

    def step(self, action):
        pass

    def sim_start_(self):
        pass


class BaseAgent(object):
    def __init__(self):
        pass

    def act(self, state):
        pass


class MotionSim(object):
    def __init__(self, glob):
        self.FRAMELENGTH = 0.02
        self.trims = []
        self.joint_handle = []
        self.dummy_h_handle = 0

        self.client_id = -1
        self.sim_step_counter = 0
        self.params = glob.params

        self.FACTOR = [1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1]

        a5 = 21.5  # мм расстояние от оси симметрии до оси сервы 5
        b5 = 18.5  # мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
        c5 = 0  # мм расстояние от оси сервы 6 до нуля Z по вертикали
        a6 = 42  # мм расстояние от оси сервы 6 до оси сервы 7
        a7 = 65.5  # мм расстояние от оси сервы 7 до оси сервы 8
        a8 = 63.8  # мм расстояние от оси сервы 8 до оси сервы 9
        a9 = 35.5  # мм расстояние от оси сервы 9 до оси сервы 10
        a10 = 25.4  # мм расстояние от оси сервы 10 до центра стопы по горизонтали
        b10 = 26.4  # мм расстояние от оси сервы 10 до низа стопы   26.4
        c10 = 12  # мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
        self.SIZES = [a5, b5, c5, a6, a7, a8, a9, a10, b10, c10]
        self.d10 = 53.4  # 53.4 расстояние по Y от центра стопы до оси робота

        limAlpha5 = [-2667, 2667]
        limAlpha6 = [-3000, 740]
        limAlpha7 = [-3555, 3260]
        limAlpha8 = [-4150, 1777]
        limAlpha9 = [-4000, 2960]
        limAlpha10 = [-2815, 600]
        self.LIM_ALPHA = [limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10]
        self.LIM_ALPHA[3][1] = 0

        self.TIK2RAD = 0.00058909
        self.slow_time = 0.0  # seconds
        self.sim_thread_cycle_in_ms = 20
        self.rotation = 0  # -45 - +45 degrees Centigrade per step + CW, - CCW.
        self.first_leg_is_right = True
        self.base_step_length = 90
        self.base_side_length = 0
        # Following paramenetrs Not recommended for change
        self.amplitude = 20  # mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
        self.fr1 = 0  # frame number for 1-st phase of gait ( two legs on floor)
        self.fr2 = 24  # frame number for 2-nd phase of gait ( one leg in air)
        self.gait_height = 190  # Distance between Center of mass and floor in walk pose
        self.step_height = 32.0  # elevation of sole over floor
        self.alpha = Alpha()
        self.exit_flag = 0
        self.neck_pan = 0
        self.xtr = 0
        self.ytr = -self.d10  # -53.4
        self.ztr = -self.gait_height
        self.xr = 0
        self.yr = 0
        self.zr = -1
        self.wr = 0
        self.xtl = 0
        self.ytl = self.d10  # 53.4
        self.ztl = -self.gait_height
        self.xl = 0
        self.yl = 0
        self.zl = -1
        self.wl = 0
        self.euler_angle = {}
        self.ACTIVEJOINTS = ['Leg_right_10', 'Leg_right_9', 'Leg_right_8', 'Leg_right_7', 'Leg_right_6', 'Leg_right_5',
                             'hand_right_4',
                             'hand_right_3', 'hand_right_2', 'hand_right_1', 'Tors1', 'Leg_left_10', 'Leg_left_9',
                             'Leg_left_8',
                             'Leg_left_7', 'Leg_left_6', 'Leg_left_5', 'hand_left_4', 'hand_left_3', 'hand_left_2',
                             'hand_left_1', 'head0', 'head12']
        self.cycle = 0
        self.number_of_cycles = 25
        self.general_step = 0
        self.total_steps = self.fr2 + self.number_of_cycles
        self.sim_backend = SimBackend(self.ACTIVEJOINTS, "Dummy_H")
        self.framestep = self.sim_thread_cycle_in_ms // 10
        self.walk_cycle_steps_each_leg = self.fr2 // self.framestep
        self.total_walk_cycle_steps = 2 * self.walk_cycle_steps_each_leg
        self.walk_cycle_step = 0

    @staticmethod
    def norm_yaw(yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:
            yaw -= 2 * math.pi
        if yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw

    @staticmethod
    def quaternion_to_euler_angle(quaternion):
        euler_angle = {}
        w,x,y,z = quaternion
        ysqr = y*y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0,t1))
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3,t4))
        euler_angle['yaw'] = math.radians(X)
        euler_angle['pitch'] = math.radians(Y)
        euler_angle['roll'] = math.radians(Z)
        return euler_angle

    @staticmethod
    def normalize_rotation(yaw):
        if abs(yaw) > 2 * math.pi:
            yaw %= (2 * math.pi)
        if yaw > math.pi:
            yaw -= (2 * math.pi)
        if yaw < -math.pi:
            yaw += (2 * math.pi)
        if yaw > 0.5:
            yaw = 0.5
        if yaw < -0.5:
            yaw = -0.5
        return yaw

    def imu_body_yaw(self):
        yaw = self.neck_pan * self.TIK2RAD + self.euler_angle['yaw']
        yaw = self.norm_yaw(yaw)
        return yaw

    def compute_alpha_for_walk(self):
        angles = []
        hands_on = True
        angles_r = self.alpha.compute_alpha_v3(self.xtr, self.ytr, self.ztr, self.xr, self.yr, self.zr, self.wr, self.SIZES, self.LIM_ALPHA)
        angles_l = self.alpha.compute_alpha_v3(self.xtl, -self.ytl, self.ztl, self.xl, -self.yl, self.zl, self.wl, self.SIZES, self.LIM_ALPHA)

        if not len(angles_r) or not len(angles_l):
            print("AAAAAAAA")
            return []

        for i in range(len(angles_r)):
            if len(angles_r) == 1:
                break
            if angles_r[0][2] < angles_r[1][2]:
                angles_r.pop(1)
            else:
                angles_r.pop(0)

        for i in range(len(angles_l)):
            if len(angles_l) == 1:
                break
            if angles_l[0][2] < angles_l[1][2]:
                angles_l.pop(1)
            else:
                angles_l.pop(0)

        if self.first_leg_is_right:
            angles_l, angles_r = angles_r, angles_l

        for j in range(6):
            angles.append(angles_l[0][j])
        if hands_on:
            angles.append(1.745)
        else:
            angles.append(0.0)
        angles.append(0.0)
        angles.append(0.0)
        if hands_on:
            angles.append(0.524 - self.xtr / 57.3)
        else:
            angles.append(0.0)
        angles.append(0.0)
        for j in range(6):
            angles.append(-angles_r[0][j])
        if hands_on:
            angles.append(-1.745)
        else:
            angles.append(0.0)
        angles.append(0.0)
        angles.append(0.0)
        if hands_on:
            angles.append(-0.524 + self.xtl / 57.3)
        else:
            angles.append(0.0)
        return angles

    def step_length_planer(self, regular_step_length, regular_side_length, framestep, hovernum1):
        xt0 = regular_step_length / 2 * self.fr2 / (self.fr2 + framestep * hovernum1)
        dy0 = regular_side_length / (self.fr2 + hovernum1 * framestep) * framestep
        dy = regular_side_length / (self.fr2 - hovernum1 * framestep) * framestep
        return xt0, dy0, dy

    def feet_action(self):
        angles = self.compute_alpha_for_walk()
        if not len(angles):
            self.exit_flag = self.exit_flag + 1
            return False
        else:
            self.sim_backend.wait_step()
            new_positions = []

            for i in range(len(angles)):
                new_positions.append(angles[i] * self.FACTOR[i])
            self.sim_backend.set_joint_positions(new_positions)
            return True

    def refresh_orientation(self):
        dummy_h_quaternion = self.sim_backend.get_imu_quaternion()
        self.euler_angle = self.quaternion_to_euler_angle(dummy_h_quaternion)

    def act(self):
        if self.general_step >= self.total_steps:
            return False

        if not self.general_step:
            self.refresh_orientation()

        if self.general_step < self.fr2:
            i = self.general_step
            amplitude = 70
            self.ztr = -223.1 + i * (223.1-self.gait_height) / self.fr2
            self.ztl = -223.1 + i * (223.1-self.gait_height) / self.fr2
            self.ytr = -self.d10 - i * amplitude / 2 / self.fr2
            self.ytl = self.d10 - i * amplitude / 2 / self.fr2
            self.feet_action()
            self.general_step += 1
        else:
            for self.walk_cycle_step in range(self.total_walk_cycle_steps):
                if self.walk_cycle_step == 0:
                    self.cycle = self.general_step - self.fr2
                    self.refresh_orientation()
                    self.rotation = self.imu_body_yaw() * 1.2
                    if self.first_leg_is_right:
                        self.rotation *= -1
                    self.rotation = self.normalize_rotation(self.rotation)

                    self.rotation = math.degrees(self.rotation)
                    self.side_length = self.base_side_length
                    self.step_length = self.base_step_length
                    self.second_step_length = self.base_step_length
                    if self.cycle == 0:
                        self.step_length = self.step_length / 3
                        self.second_step_length = self.step_length / 3
                    if self.cycle == 1:
                        self.step_length = self.step_length / 3 * 2
                        self.second_step_length = self.step_length / 3 * 2

                    self.rotation = -self.rotation / 286
                    self.alpha01 = math.pi / self.fr2
                    self.hovernum = 6  # number of steps hovering over take off + landing points
                    self.xr_old, self.xl_old, self.yr_old, self.yl_old = self.xr, self.xl, self.yr, self.yl
                    # correction of sole skew depending on side angle of body when step pushes land
                    self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
                    # correction of body tilt forward
                    self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']  #
                    self.wr_old = self.wr
                    self.wl_old = self.wl
                    self.wr_target = - self.rotation
                    self.wl_target = - self.rotation
                    self.xt0, self.dy0, self.dy = self.step_length_planer(self.step_length, self.side_length, self.framestep,
                                                           self.hovernum)
                    self.ztl = -self.gait_height
                    xtl0 = self.xtl
                    xtr0 = self.xtr
                    self.xtl1 = -self.xt0
                    self.xtr1 = self.xt0
                    self.dx0 = (self.xtl1 - xtl0) * self.framestep / self.fr2
                    self.dx = (self.xtr1 - xtr0) * self.framestep / self.fr2 * (self.fr2 + self.hovernum * self.framestep) / (
                        self.fr2 - self.hovernum * self.framestep)
                if self.walk_cycle_step < self.walk_cycle_steps_each_leg:
                    iii = self.walk_cycle_step * self.framestep
                    if 2 * self.framestep < iii < self.fr2 - 4 * self.framestep:
                        self.xt0, self.dy0, self.dy = self.step_length_planer(self.step_length, self.side_length, self.framestep, self.hovernum)

                        self.xtl1 = -self.xt0
                        self.dx0 = (self.xtl1 - self.xtl) * self.framestep / (self.fr2 - iii)
                        self.dx = (- self.xtr - self.xtl - self.dx0 * ((self.fr2 - iii) / self.framestep + 3)) / (
                                (self.fr2 - iii) / self.framestep - 3)
                    S = self.amplitude / 2 * math.sin(self.alpha01 * iii)
                    self.ytr = -S - self.d10
                    self.ytl = -S + self.d10
                    self.ztr = -self.gait_height
                    if iii == 0:
                        self.ztr = -self.gait_height + self.step_height / 3
                    elif iii == self.framestep:
                        self.ztr = -self.gait_height + self.step_height * 2 / 3
                    elif iii == self.fr2 - self.framestep:
                        self.ztr = -self.gait_height + self.step_height / 4
                    elif iii == self.fr2 - 2 * self.framestep:
                        self.ztr = -self.gait_height + self.step_height * 2 / 4
                    elif iii == self.fr2 - 3 * self.framestep:
                        self.ztr = -self.gait_height + self.step_height * 3 / 4
                    else:
                        self.ztr = -self.gait_height + self.step_height
                    if iii == 0 or iii == self.framestep or iii == 2 * self.framestep:
                        self.xtr += self.dx0
                        self.ytr = -64 + self.dy0 * iii
                    elif iii == self.fr2 - self.framestep or iii == self.fr2 - 2 * self.framestep or iii == self.fr2 - 3 * self.framestep:
                        self.xtr += self.dx0
                        self.ytr = -64 + self.dy0 * 3 * self.framestep - self.dy * (self.fr2 - 3 * self.framestep) / 2 + self.dy0 * (
                                iii - (self.fr2 - 3 * self.framestep))
                    else:
                        self.xtr += self.dx
                        self.ytr = - 64 + self.dy0 * 3 * self.framestep - self.dy * iii / 2
                        self.wr = self.wr_old + (self.wr_target - self.wr_old) * (iii) / (self.fr2 - self.hovernum * self.framestep)
                        self.wl = self.wl_old + (self.wl_target - self.wl_old) * (iii) / (self.fr2 - self.hovernum * self.framestep)

                    self.xtl += self.dx0
                    self.ytl = -S + self.d10 + self.dy0 * iii
                else:
                    if self.walk_cycle_step == self.walk_cycle_steps_each_leg:
                        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']  #

                        self.xt0, self.dy0, self.dy = self.step_length_planer(self.second_step_length, self.side_length, self.framestep, self.hovernum)
                        self.xtr1 = self.xtr
                        self.ztr = -self.gait_height
                        if self.cycle == self.number_of_cycles - 1:
                            xtr2 = 0
                        else:
                            xtr2 = -self.xt0
                        self.dx0 = (xtr2 - self.xtr1) * self.framestep / self.fr2
                        self.dx = - self.dx0 * (self.fr2 + self.hovernum * self.framestep) / (self.fr2 - self.hovernum * self.framestep)
                    iii = (self.walk_cycle_step - self.walk_cycle_steps_each_leg) * self.framestep
                    if 2 * self.framestep < iii < self.fr2 - 4 * self.framestep:
                        self.xt0, self.dy0, self.dy = self.step_length_planer(self.second_step_length, self.side_length, self.framestep, self.hovernum)
                        if self.cycle == self.number_of_cycles - 1:
                            xtr2 = 0
                        else:
                            xtr2 = -self.xt0
                        self.dx0 = (xtr2 - self.xtr) * self.framestep / (self.fr2 - iii)
                        self.dx = (- self.xtr - self.xtl - self.dx0 * ((self.fr2 - iii) / self.framestep + 3)) / (
                                (self.fr2 - iii) / self.framestep - 3)
                    S = -self.amplitude / 2 * math.sin(self.alpha01 * iii)
                    self.ytr = -S - self.d10
                    self.ytl = -S + self.d10
                    self.ztl = -self.gait_height
                    if iii == 0:
                        self.ztl = -self.gait_height + self.step_height / 3
                    elif iii == self.framestep:
                        self.ztl = -self.gait_height + self.step_height * 2 / 3
                    elif iii == self.fr2 - self.framestep:
                        self.ztl = -self.gait_height + self.step_height / 4
                    elif iii == self.fr2 - 2 * self.framestep:
                        self.ztl = -self.gait_height + self.step_height * 2 / 4
                    elif iii == self.fr2 - 3 * self.framestep:
                        self.ztl = -self.gait_height + self.step_height * 3 / 4
                    else:
                        self.ztl = -self.gait_height + self.step_height
                    if self.cycle == self.number_of_cycles - 1:
                        if iii == (self.fr2 - self.framestep):
                            self.ztl = -self.gait_height
                            self.ytl = S + self.d10
                    if iii == 0 or iii == self.framestep or iii == 2 * self.framestep:
                        self.xtl += self.dx0
                        self.ytl = S + self.d10 + self.dy0 * iii
                    elif iii == self.fr2 - self.framestep or iii == self.fr2 - 2 * self.framestep or iii == self.fr2 - 3 * self.framestep:
                        self.xtl += self.dx0
                        self.ytl = S + 64 + self.dy0 * 3 * self.framestep - self.dy * (self.fr2 - self.hovernum * self.framestep) + self.dy0 * (
                                iii - (self.fr2 - 3 * self.framestep))
                    else:
                        self.xtl += self.dx
                        self.ytl = S + 64 + self.dy0 * 3 * self.framestep - self.dy * (iii - 3 * self.framestep)
                        self.wr = self.wr_target * (1 - iii / (self.fr2 - self.hovernum * self.framestep) * 2)
                        self.wl = self.wl_target * (1 - iii / (self.fr2 - self.hovernum * self.framestep) * 2)
                    self.xtr += self.dx0
                    self.ytr += self.dy0
                    if self.ytl < 54:
                        self.ytl = 54
                self.feet_action()
            self.walk_cycle_step = self.total_walk_cycle_steps
            if self.walk_cycle_step == self.total_walk_cycle_steps:
                self.xr, self.xl, self.yr, self.yl = self.xr_old, self.xl_old, self.yr_old, self.yl_old
                self.general_step += 1
        return True


if __name__ == "__main__":
    print('This is not main module!')
