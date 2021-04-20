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
        self.step_length = 0.0  # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
        self.side_length = 0.0  # -20 - +20. Side step length to right (+) and to left (-)
        self.rotation = 0  # -45 - +45 degrees Centigrade per step + CW, - CCW.
        self.first_leg_is_right = True

        # Following paramenetrs Not recommended for change
        self.amplitude = 32  # mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
        self.fr1 = 8  # frame number for 1-st phase of gait ( two legs on floor)
        self.fr2 = 12  # frame number for 2-nd phase of gait ( one leg in air)
        self.gait_height = 190  # Distance between Center of mass and floor in walk pose
        self.step_height = 32.0  # elevation of sole over floor
        self.init_poses = 400 // self.sim_thread_cycle_in_ms
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
        self.sim_backend = SimBackend(self.ACTIVEJOINTS, "Dummy_H")

    def imu_body_yaw(self):
        yaw = self.neck_pan * self.TIK2RAD + self.euler_angle['yaw']
        yaw = self.norm_yaw(yaw)
        return yaw

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

    def step_length_planer(self, regular_step_length, regular_side_length, framestep, hovernum):
        xt0 = regular_step_length / 2 * self.fr2 / (self.fr2 + framestep * hovernum)
        dy0 = regular_side_length / (self.fr2 + hovernum * framestep) * framestep
        dy = regular_side_length / (self.fr2 - hovernum * framestep) * framestep
        return xt0, dy0, dy

    def feet_action(self):
        angles = self.compute_alpha_for_walk()
        if not len(angles):
            self.exit_flag = self.exit_flag + 1
            return False
        else:
            self.wait_sim_step()
            new_positions = []
            for i in range(len(angles)):
                new_positions.append(angles[i] * self.FACTOR[i] + self.trims[i])
            self.sim_backend.set_joint_positions(new_positions)
            return True

    def walk_initial_pose(self):
        self.xtr = self.xtl = 0
        amplitude = 70
        for j in range(self.init_poses):
            self.ztr = -223.1 + j * (223.1-self.gait_height) / self.init_poses
            self.ztl = -223.1 + j * (223.1-self.gait_height) / self.init_poses
            self.ytr = -self.d10 - j * amplitude / 2 / self.init_poses
            self.ytl = self.d10 - j * amplitude / 2 / self.init_poses
            self.feet_action()

    def walk_cycle(self, step_length, side_length, rotation, cycle, number_of_cycles, second_step_length=1000):
        self.step_length = step_length
        self.side_length = side_length
        self.rotation = math.degrees(rotation)

        rotation = -self.rotation/286
        alpha01 = math.pi/self.fr2
        frameNumberPerStep = self.fr2
        framestep = self.sim_thread_cycle_in_ms//10
        hovernum = 6     # number of steps hovering over take off + landing points
        xt0 = self.step_length /2 * self.fr2 / (self.fr2 + framestep * hovernum)
        xtr0 = - self.step_length /2 * self.fr2 / (self.fr2 + framestep * hovernum)
        dx0_typical = self.step_length/(self.fr2+ hovernum * framestep)*framestep        # CoM propulsion forward per framestep
        dy0 = self.side_length / (self.fr2 + hovernum * framestep) * framestep        # CoM propulsion sideways per framestep
        dy = self.side_length /(self.fr2 - hovernum * framestep) * framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        wr_old = self.wr
        wl_old = self.wl
        wr_target = - rotation
        wl_target = - rotation

        xt0, dy0, dy = self.step_length_planer(self.step_length, self.side_length, framestep, hovernum)
        self.ztl = -self.gait_height
        xtl0 = self.xtl
        xtr0 = self.xtr
        xtl1 = -xt0
        xtr1 = xt0
        dx0 = (xtl1 - xtl0) * framestep / self.fr2
        dx = (xtr1 - xtr0) * framestep / self.fr2 * (self.fr2 + hovernum * framestep)/ (self.fr2 - hovernum * framestep)
        for iii in range(0, frameNumberPerStep, framestep):
            start1 = 0
            if 2 * framestep < iii <  self.fr2 - 4 * framestep:
                xt0, dy0, dy = self.step_length_planer(self.step_length, self.side_length, framestep, hovernum)

                xtl1 = -xt0
                xtr1 = xt0
                if cycle == 0:
                    xtl0 = 0
                    xtr0 = 0
                else:
                    xtl0 = xt0
                    xtr0 = -xt0
                dx0 = (xtl1 - self.xtl) * framestep / (self.fr2 - iii)
                dx = (- self.xtr - self.xtl - dx0  * ( (self.fr2 - iii)/ framestep + 3)) /( (self.fr2 - iii)/ framestep - 3)
            S = self.amplitude/2 *math.sin(alpha01 * iii )
            self.ytr = -S - self.d10
            self.ytl = -S + self.d10
            self.ztr = -self.gait_height
            if iii== 0 :
                self.ztr = -self.gait_height + self.step_height / 3
            elif iii== framestep:
                self.ztr = -self.gait_height + self.step_height * 2 / 3
            elif iii==self.fr2 - framestep:
                self.ztr = -self.gait_height + self.step_height / 4
            elif iii==self.fr2 - 2 * framestep:
                self.ztr = -self.gait_height + self.step_height * 2 / 4
            elif iii==self.fr2 - 3 * framestep:
                self.ztr = -self.gait_height + self.step_height * 3 / 4
            else:
                self.ztr = -self.gait_height + self.step_height
            if iii == 0 or iii == framestep or iii == 2 * framestep:
                self.xtr += dx0
                self.ytr = -64 + dy0 * iii
            elif iii == self.fr2 - framestep or iii == self.fr2 - 2 * framestep or iii == self.fr2 - 3 * framestep:
                self.xtr += dx0
                self.ytr = -64 + dy0 * 3 * framestep - dy*(self.fr2 - 3 * framestep)/2 + dy0 * (iii - (self.fr2 - 3 * framestep))
            else:
                self.xtr += dx
                self.ytr = - 64 + dy0 * 3 * framestep - dy*iii/2
                self.wr = wr_old + (wr_target - wr_old) * (iii)/(self.fr2 - hovernum * framestep)
                self.wl = wl_old + (wl_target - wl_old) * (iii)/(self.fr2- hovernum * framestep)

            self.xtl += dx0
            self.ytl = -S + self.d10 + dy0 * iii
            self.feet_action()

        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #

        xt0, dy0, dy = self.step_length_planer(second_step_length, self.side_length, framestep, hovernum)
        xtl1  = self.xtl
        xtr1  = self.xtr
        self.ztr = -self.gait_height
        if cycle == number_of_cycles - 1:
            xtl2 = 0
            xtr2 = 0
        else:
            xtl2 = xt0
            xtr2 = -xt0
        dx0 = (xtr2 - xtr1) * framestep / self.fr2
        dx = - dx0 * (self.fr2 + hovernum * framestep)/ (self.fr2 - hovernum * framestep)
        for iii in range(0, frameNumberPerStep, framestep):
            start1 = 0
            if 2 * framestep < iii <  self.fr2 - 4 * framestep:
                xt0, dy0, dy = self.step_length_planer(second_step_length, self.side_length, framestep, hovernum)
                xtl1 = -xt0
                xtr1 = xt0
                if cycle == number_of_cycles - 1:
                    xtl2 = 0
                    xtr2 = 0
                else:
                    xtl2 = xt0
                    xtr2 = -xt0
                dx0 = (xtr2 - self.xtr) * framestep / (self.fr2 - iii)
                dx = (- self.xtr - self.xtl - dx0  * ( (self.fr2 - iii)/ framestep + 3)) /( (self.fr2 - iii)/ framestep - 3)
            S = -self.amplitude/2 *math.sin(alpha01 * iii)
            self.ytr = -S - self.d10
            self.ytl = -S + self.d10
            self.ztl = -self.gait_height
            if iii == 0:
                self.ztl = -self.gait_height + self.step_height / 3
            elif iii == framestep:
                self.ztl = -self.gait_height + self.step_height * 2 / 3
            elif iii == self.fr2 - framestep:
                self.ztl = -self.gait_height + self.step_height / 4
            elif iii == self.fr2 - 2 * framestep:
                self.ztl = -self.gait_height + self.step_height * 2 / 4
            elif iii == self.fr2 - 3 * framestep:
                self.ztl = -self.gait_height + self.step_height * 3 / 4
            else:
                self.ztl = -self.gait_height + self.step_height
            if cycle == number_of_cycles - 1:
                if iii== (self.fr2 - framestep):
                    self.ztl = -self.gait_height
                    self.ytl = S + self.d10
            if iii == 0 or iii == framestep or iii == 2 * framestep :
                self.xtl += dx0
                self.ytl = S + self.d10 + dy0 * iii
            elif iii == self.fr2 - framestep or iii == self.fr2 - 2 * framestep or iii == self.fr2 - 3 * framestep :
                self.xtl += dx0
                self.ytl = S + 64 + dy0 * 3 * framestep - dy * (self.fr2 - hovernum * framestep) + dy0 * (iii - (self.fr2 - 3 * framestep))
            else:
                self.xtl += dx
                self.ytl = S + 64 + dy0 * 3 * framestep - dy * (iii - 3 * framestep)
                self.wr = wr_target * (1 - iii / (self.fr2 - hovernum * framestep) * 2)
                self.wl = wl_target * (1 - iii / (self.fr2 - hovernum * framestep) * 2)
            self.xtr += dx0
            self.ytr += dy0
            if self.ytl < 54:
                self.ytl = 54
            self.feet_action()
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old

    def refresh_orientation(self):
        dummy_h_quaternion = self.sim_backend.get_imu_quaternion()
        self.euler_angle = self.quaternion_to_euler_angle(dummy_h_quaternion)

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

    def wait_sim_step(self):
        self.sim_backend.wait_step()

    def sim_start(self):
        self.sim_backend.start()
        self.trims = self.sim_backend.get_joint_positions()

    def sim_stop(self):
        self.sim_backend.stop()
        self.sim_backend.set_joint_positions(self.trims)

    def sim_disable(self):
        self.sim_backend.disable()


if __name__ == "__main__":
    print('This is not main module!')
