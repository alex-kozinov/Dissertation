#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import math
import sys
import time

import numpy as np


class Alpha(object):
    def compute_alpha_v3(self, xt,yt,zt,x,y,z,w, sizes, limAlpha):
        from math import sqrt,cos,sin,asin,fabs,tan,atan
        #t1_start =time.perf_counter_ns()
        a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 = sizes
        limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10 = limAlpha
        alpha5 = w
        cos5 = math.cos(alpha5)
        sin5 = math.sin(alpha5)
        nor = math.sqrt(x*x+y*y+z*z)
        x = x/nor
        y = y/nor
        z = z/nor
        xtp = xt * cos5 + (yt + a5) * sin5
        ytp = (yt + a5) * cos5 - xt * sin5
        ztp = zt
        xp =  x * cos5 + y * sin5
        yp = y * cos5 - x * sin5
        zp = z
        var = [-1,1]
        angles = []
        lim1a= limAlpha6[0]*0.00058909
        lim2a = limAlpha6[1]*0.00058909
        ind = 1
        step1 = (lim2a-lim1a)/10
        testalpha6 =[]
        for i in range (11):
            alpha6 = lim1a+i*step1
            cos= math.cos(alpha6)
            sin= math.sin(alpha6)
            testalpha6.append( ((ytp+b5)*cos+ztp*sin -c10)*((yp*cos+zp*sin)**2-
                (zp*cos-yp*sin)**2 -xp*xp)-a10-b10*(yp*cos+zp*sin)/math.sqrt((zp*cos-yp*sin)**2+xp*xp))

        points = []
        for i in range(10):
            if (testalpha6[i]>0 and testalpha6[i+1]<0)or(testalpha6[i]<0 and testalpha6[i+1]>0): points.append(i)
        k=0
        if len(points)==0:
            for i in range(11):
                if (math.fabs(testalpha6[i]) < math.fabs(testalpha6[k])): k=i
            if k==10: points.append(9)
            else:
                if (math.fabs(testalpha6[k-1]) < math.fabs(testalpha6[k+1])): points.append(k-1)
                else: points.append(k)
        alpha6m = []
        for j in range(len(points)):
            lim1=lim1a+points[j]*step1
            lim2=lim1+step1
            while (True):
                step = (lim2-lim1)/10
                testalpha6 =[]
                for i in range (11):
                    alpha6 = lim1+i*step
                    cos= math.cos(alpha6)
                    sin= math.sin(alpha6)
                    testalpha6.append( ((ytp+b5)*cos+ztp*sin-c10)*((yp*cos+zp*sin)**2-
                        (zp*cos-yp*sin)**2 -xp*xp)-a10-b10*(yp*cos+zp*sin)/math.sqrt((zp*cos-yp*sin)**2+xp*xp))
                k=0
                for i in range(11):
                    if (math.fabs(testalpha6[i]) < math.fabs(testalpha6[k])): k = i
                if k==0: k2=1
                elif k==10: k2 = 9
                else:
                    if (math.fabs(testalpha6[k-1]) < math.fabs(testalpha6[k+1])): k2=k-1
                    else: k2=k+1
                alpha6 = lim1+k*step
                if k>k2:
                    lim1 = lim1+k2*step
                    lim2 = lim1+ step
                else:
                    lim1 = lim1+k*step
                    lim2 = lim1+ step
                if (lim2-lim1 < 0.00025): break
                ind = ind + 1
                if ind> (limAlpha6[1]- limAlpha6[0]): break
            alpha6m.append(alpha6)
        alpha10m =[]
        kk=0

        for i in range (len(alpha6m)):
            tan6 = math.tan(alpha6m[i-kk])
            alpha10 = math.atan((-yp-zp*tan6)/math.sqrt((zp-yp*tan6)**2+xp*xp*(1+tan6*tan6)))
            if limAlpha10[0] < alpha10*1698 and alpha10*1698<limAlpha10[1]: alpha10m.append(alpha10)
            else:
                alpha6m.pop(i-kk)
                kk=kk+1
        kk=0

        for ii in range (len(alpha6m)):
            cos6 = math.cos(alpha6m[ii-kk])
            sin6 = math.sin(alpha6m[ii-kk])
            alpha987 = math.atan(-xp/(zp*cos6- yp*sin6))
            sin987 = math.sin(alpha987)
            cos987 = math.cos(alpha987)
            K1 = a6*sin987+xtp*cos987+(ztp*cos6-(ytp+b5)*sin6)*sin987
            K2 = a9+a6*cos987+(ztp*cos6-(ytp+b5)*sin6)*cos987-xtp*sin987+b10/math.cos(alpha10m[ii-kk])+((ytp+b5)*cos6+ztp*sin6-c10)*math.tan(alpha10m[ii-kk])
            m = (K1*K1+K2*K2+a8*a8-a7*a7)/(2*a8)


            temp1 = K1*K1*m*m-(K1*K1+K2*K2)*(m*m-K2*K2)
            if temp1>=0 :
                temp2 = (-K1*m + math.sqrt(temp1))/(K1*K1+K2*K2)
                temp3 = (-K1*m - math.sqrt(temp1))/(K1*K1+K2*K2)
                if math.fabs(temp2) <= 1 and math.fabs(temp3) <= 1:
                    alpha91 = math.asin(temp2)
                    alpha92 = math.asin(temp3)
                else:
                    alpha6m.pop(ii-kk)
                    alpha10m.pop(ii-kk)
                    kk=kk+1
                    continue
            else:
                alpha6m.pop(ii-kk)
                alpha10m.pop(ii-kk)
                kk=kk+1
                continue
            alpha81 = math.atan((K1+a8*math.sin(alpha91))/(K2+a8*math.cos(alpha91))) - alpha91
            alpha82 = math.atan((K1+a8*math.sin(alpha92))/(K2+a8*math.cos(alpha92))) - alpha92
            alpha71 = alpha91+alpha81- alpha987
            alpha72 = alpha92+alpha82- alpha987
            temp71 = alpha71*1698<limAlpha7[0] or alpha71*1698>limAlpha7[1]
            temp72 = alpha72*1698<limAlpha7[0] or alpha72*1698>limAlpha7[1]
            temp81 = alpha81*1698<limAlpha8[0] or alpha81*1698>limAlpha8[1]
            temp82 = alpha82*1698<limAlpha8[0] or alpha82*1698>limAlpha8[1]
            temp91 = alpha91*1698<limAlpha9[0] or alpha91*1698>limAlpha9[1]
            temp92 = alpha92*1698<limAlpha9[0] or alpha92*1698>limAlpha9[1]
            if (temp71 and temp72) or (temp81 and temp82) or (temp91 and temp92) or ((temp71 or temp81 or temp91) and (temp72 or temp82 or temp92)):
                alpha6m.pop(ii-kk)
                alpha10m.pop(ii-kk)
                kk=kk+1
                continue
            else:
                if not (temp71 or temp81 or temp91):
                    ang =()
                    ang =alpha10m[ii-kk],alpha91,alpha81,alpha71,alpha6m[ii-kk],alpha5
                    angles.append(ang)
                if not (temp72 or temp82 or temp92):
                    ang =()
                    ang =alpha10m[ii-kk],alpha92,alpha82,alpha72,alpha6m[ii-kk],alpha5
                    angles.append(ang)
        return angles


class SimBackend(object):
    def __init__(self, joint_names, imu_name, sim_path):
        sys.path.append(sim_path)
        import sim
        self.sim_lib = sim
        self.joint_names = joint_names
        self.imu_name = imu_name
        self.joint_handles = []
        self.client_id = -1
        self.sim_step_counter = 0
        self.imu_handle = None
        self.is_first_imu_call = True

    def start(self):
        self.sim_lib.simxFinish(-1)  # just in case, close all opened connections
        sim_thread_cycle_in_ms = 5
        self.client_id = self.sim_lib.simxStart('127.0.0.1', -19997, True, True, 5000, sim_thread_cycle_in_ms)

        _, self.imu_handle = self.sim_lib.simxGetObjectHandle(self.client_id, self.imu_name, self.sim_lib.simx_opmode_blocking)
        for i, join_name in enumerate(self.joint_names):
            _, handle = self.sim_lib.simxGetObjectHandle(self.client_id, join_name, self.sim_lib.simx_opmode_blocking)
            self.joint_handles.append(handle)

        self.sim_lib.simxGetIntegerParameter(self.client_id, self.sim_lib.sim_intparam_program_version, self.sim_lib.simx_opmode_streaming)
        time.sleep(0.1)
        self.sim_lib.simxStartSimulation(self.client_id, self.sim_lib.simx_opmode_oneshot)

    def wait_step(self):
        while True:
            self.sim_lib.simxGetIntegerParameter(self.client_id, self.sim_lib.sim_intparam_program_version, self.sim_lib.simx_opmode_buffer)
            tim = self.sim_lib.simxGetLastCmdTime(self.client_id)

            if tim > self.sim_step_counter:
                self.sim_step_counter = tim
                break
            time.sleep(0.001)

    def stop(self):
        self.sim_lib.simxStopSimulation(self.client_id, self.sim_lib.simx_opmode_oneshot)

    def disable(self):
        time.sleep(0.2)
        self.sim_lib.simxFinish(self.client_id)

    def set_joint_positions(self, positions):
        self.sim_lib.simxPauseCommunication(self.client_id, True)
        for position, joint_handle in zip(positions, self.joint_handles):
            self.sim_lib.simxSetJointTargetPosition(
                self.client_id,
                joint_handle,
                position,
                self.sim_lib.simx_opmode_oneshot
            )
        self.sim_lib.simxPauseCommunication(self.client_id, False)

    def get_joint_positions(self):
        positions = []
        for i, joint_handle in enumerate(self.joint_handles):
            _, position = self.sim_lib.simxGetJointPosition(self.client_id, joint_handle, self.sim_lib.simx_opmode_blocking)
            positions.append(position)

        return positions

    def get_imu_quaternion(self):
        # if self.is_first_imu_call:
        #     self.is_first_imu_call = False
        #     self.get_imu_quaternion()
        #     self.get_imu_quaternion()
        _, dummy_h_quaternion = self.sim_lib.simxGetObjectQuaternion(self.client_id, self.imu_handle, -1, self.sim_lib.simx_opmode_streaming)
        return dummy_h_quaternion


class DirectEnvironment(object):
    def __init__(self, joint_names, imu_name, sim_path):
        self.sim_backend = SimBackend(joint_names, imu_name, sim_path)
        self.joint_positions = None

    def __del__(self):
        self.sim_backend.stop()
        self.sim_backend.disable()

    def reset(self):
        self.sim_backend.start()
        self.joint_positions = self.sim_backend.get_joint_positions()
        return (self.joint_positions, self.sim_backend.get_imu_quaternion())

    def step(self, action):
        self.sim_backend.wait_step()
        if action:
            self.joint_positions = action
        self.sim_backend.set_joint_positions(self.joint_positions)
        return (self.joint_positions, self.sim_backend.get_imu_quaternion()), 0


class MotionSim(object):
    def __init__(self, random_mode=False, foot_only_mode=False):
        self.FRAMELENGTH = 0.02
        self.trims = []
        self.joint_handle = []
        self.dummy_h_handle = 0

        self.client_id = -1
        self.sim_step_counter = 0
        self._foot_only_mode = foot_only_mode
        if random_mode:
            self.params = {
                "BODY_TILT_AT_WALK": np.random.uniform(low=0.001, high=0.1),
                "SOLE_LANDING_SKEW": np.random.uniform(low=0.035, high=0.085),
                "FIRST_LEG_IS_RIGHT": np.random.choice([True, False]),
                # "BASE_STEP_LENGTH": np.random.uniform(low=20., high=120),
                # "BASE_SIDE_LENGTH": np.random.uniform(low=0., high=40),
                "BASE_STEP_LENGTH": np.random.uniform(low=40., high=100),
                "BASE_SIDE_LENGTH": np.random.uniform(low=0., high=20),
                "AMPLITUDE": np.random.uniform(low=5., high=35),
                "FR2": np.random.randint(low=18, high=24),
                "GAIT_HEIGHT": np.random.uniform(low=180., high=200),
                "STEP_HEIGHT": np.random.uniform(low=23., high=45),
            }
        else:
            self.params = {
                "BODY_TILT_AT_WALK": 0.03,
                "SOLE_LANDING_SKEW": 0.06,
                "FIRST_LEG_IS_RIGHT": False,
                "BASE_STEP_LENGTH": 90,
                "BASE_SIDE_LENGTH": 0,
                "AMPLITUDE": 20,
                "FR2": 24,
                "GAIT_HEIGHT": 190,
                "STEP_HEIGHT": 32.0,
            }
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
        # self.params["BASE_STEP_LENGTH"] = 90
        # self.params["BASE_SIDE_LENGTH"] = 0
        # # Following paramenetrs Not recommended for change
        # self.params["AMPLITUDE"] = 20  # mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
        # self.params["FR2"] = 24  # frame number for 2-nd phase of gait ( one leg in air)
        # self.params["GAIT_HEIGHT"] = 190  # Distance between Center of mass and floor in walk pose
        # self.params["STEP_HEIGHT"] = 32.0  # elevation of sole over floor
        self.alpha = Alpha()
        self.exit_flag = 0
        self.neck_pan = 0
        self.xtr = 0
        self.ytr = -self.d10  # -53.4
        self.ztr = -self.params["GAIT_HEIGHT"]
        self.xr = 0
        self.yr = 0
        self.zr = -1
        self.wr = 0
        self.xtl = 0
        self.ytl = self.d10  # 53.4
        self.ztl = -self.params["GAIT_HEIGHT"]
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
                             'hand_left_1']
        self.cycle = 0
        self.number_of_cycles = 50
        self.general_step = 0
        self.total_steps = self.params["FR2"] + self.number_of_cycles
        self.framestep = self.sim_thread_cycle_in_ms // 10
        self.walk_cycle_steps_each_leg = self.params["FR2"] // self.framestep
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

        if self.params["FIRST_LEG_IS_RIGHT"]:
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
        xt0 = regular_step_length / 2 * self.params["FR2"] / (self.params["FR2"] + framestep * hovernum1)
        dy0 = regular_side_length / (self.params["FR2"] + hovernum1 * framestep) * framestep
        dy = regular_side_length / (self.params["FR2"] - hovernum1 * framestep) * framestep
        return xt0, dy0, dy

    def get_feet_action(self):
        angles = self.compute_alpha_for_walk()
        new_positions = []
        if not len(angles):
            self.exit_flag = self.exit_flag + 1
        else:
            for i in range(len(angles)):
                new_positions.append(angles[i] * self.FACTOR[i])

        result_positions = []
        if self._foot_only_mode and len(new_positions):
            for i, joint_name in enumerate(self.ACTIVEJOINTS):
                if joint_name == "Tors1" or "Leg" in joint_name:
                    result_positions.append(new_positions[i])
        else:
            result_positions = new_positions
        return result_positions

    def refresh_orientation(self):
        dummy_h_quaternion = self.imu_quaternion
        self.euler_angle = self.quaternion_to_euler_angle(dummy_h_quaternion)

    def act(self, imu_quaternion):
        self.imu_quaternion = imu_quaternion
        if self.general_step >= self.total_steps:
            return None

        if not self.general_step:
            self.refresh_orientation()

        if self.general_step < self.params["FR2"]:
            i = self.general_step
            amplitude = 70
            self.ztr = -223.1 + i * (223.1-self.params["GAIT_HEIGHT"]) / self.params["FR2"]
            self.ztl = -223.1 + i * (223.1-self.params["GAIT_HEIGHT"]) / self.params["FR2"]
            self.ytr = -self.d10 - i * amplitude / 2 / self.params["FR2"]
            self.ytl = self.d10 - i * amplitude / 2 / self.params["FR2"]

            new_positions = self.get_feet_action()

            self.general_step += 1
        else:
            if self.walk_cycle_step == 0:
                self.cycle = self.general_step - self.params["FR2"]
                self.refresh_orientation()
                self.rotation = self.imu_body_yaw() * 1.2
                if self.params["FIRST_LEG_IS_RIGHT"]:
                    self.rotation *= -1
                self.rotation = self.normalize_rotation(self.rotation)

                self.rotation = math.degrees(self.rotation)
                self.side_length = self.params["BASE_SIDE_LENGTH"]
                self.step_length = self.params["BASE_STEP_LENGTH"]
                self.second_step_length = self.params["BASE_STEP_LENGTH"]
                if self.cycle == 0:
                    self.step_length = self.step_length / 3
                    self.second_step_length = self.step_length / 3
                if self.cycle == 1:
                    self.step_length = self.step_length / 3 * 2
                    self.second_step_length = self.step_length / 3 * 2

                self.rotation = -self.rotation / 286
                self.alpha01 = math.pi / self.params["FR2"]
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
                self.ztl = -self.params["GAIT_HEIGHT"]
                xtl0 = self.xtl
                xtr0 = self.xtr
                self.xtl1 = -self.xt0
                self.xtr1 = self.xt0
                self.dx0 = (self.xtl1 - xtl0) * self.framestep / self.params["FR2"]
                self.dx = (self.xtr1 - xtr0) * self.framestep / self.params["FR2"] * (self.params["FR2"] + self.hovernum * self.framestep) / (
                    self.params["FR2"] - self.hovernum * self.framestep)
            if self.walk_cycle_step < self.walk_cycle_steps_each_leg:
                iii = self.walk_cycle_step * self.framestep
                if 2 * self.framestep < iii < self.params["FR2"] - 4 * self.framestep:
                    self.xt0, self.dy0, self.dy = self.step_length_planer(self.step_length, self.side_length, self.framestep, self.hovernum)

                    self.xtl1 = -self.xt0
                    self.dx0 = (self.xtl1 - self.xtl) * self.framestep / (self.params["FR2"] - iii)
                    self.dx = (- self.xtr - self.xtl - self.dx0 * ((self.params["FR2"] - iii) / self.framestep + 3)) / (
                            (self.params["FR2"] - iii) / self.framestep - 3)
                S = self.params["AMPLITUDE"] / 2 * math.sin(self.alpha01 * iii)
                self.ytr = -S - self.d10
                self.ytl = -S + self.d10
                self.ztr = -self.params["GAIT_HEIGHT"]
                if iii == 0:
                    self.ztr = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] / 3
                elif iii == self.framestep:
                    self.ztr = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] * 2 / 3
                elif iii == self.params["FR2"] - self.framestep:
                    self.ztr = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] / 4
                elif iii == self.params["FR2"] - 2 * self.framestep:
                    self.ztr = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] * 2 / 4
                elif iii == self.params["FR2"] - 3 * self.framestep:
                    self.ztr = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] * 3 / 4
                else:
                    self.ztr = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"]
                if iii == 0 or iii == self.framestep or iii == 2 * self.framestep:
                    self.xtr += self.dx0
                    self.ytr = -64 + self.dy0 * iii
                elif iii == self.params["FR2"] - self.framestep or iii == self.params["FR2"] - 2 * self.framestep or iii == self.params["FR2"] - 3 * self.framestep:
                    self.xtr += self.dx0
                    self.ytr = -64 + self.dy0 * 3 * self.framestep - self.dy * (self.params["FR2"] - 3 * self.framestep) / 2 + self.dy0 * (
                            iii - (self.params["FR2"] - 3 * self.framestep))
                else:
                    self.xtr += self.dx
                    self.ytr = - 64 + self.dy0 * 3 * self.framestep - self.dy * iii / 2
                    self.wr = self.wr_old + (self.wr_target - self.wr_old) * (iii) / (self.params["FR2"] - self.hovernum * self.framestep)
                    self.wl = self.wl_old + (self.wl_target - self.wl_old) * (iii) / (self.params["FR2"] - self.hovernum * self.framestep)

                self.xtl += self.dx0
                self.ytl = -S + self.d10 + self.dy0 * iii
            else:
                if self.walk_cycle_step == self.walk_cycle_steps_each_leg:
                    self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']  #

                    self.xt0, self.dy0, self.dy = self.step_length_planer(self.second_step_length, self.side_length, self.framestep, self.hovernum)
                    self.xtr1 = self.xtr
                    self.ztr = -self.params["GAIT_HEIGHT"]
                    if self.cycle == self.number_of_cycles - 1:
                        xtr2 = 0
                    else:
                        xtr2 = -self.xt0
                    self.dx0 = (xtr2 - self.xtr1) * self.framestep / self.params["FR2"]
                    self.dx = - self.dx0 * (self.params["FR2"] + self.hovernum * self.framestep) / (self.params["FR2"] - self.hovernum * self.framestep)
                iii = (self.walk_cycle_step - self.walk_cycle_steps_each_leg) * self.framestep
                if 2 * self.framestep < iii < self.params["FR2"] - 4 * self.framestep:
                    self.xt0, self.dy0, self.dy = self.step_length_planer(self.second_step_length, self.side_length, self.framestep, self.hovernum)
                    if self.cycle == self.number_of_cycles - 1:
                        xtr2 = 0
                    else:
                        xtr2 = -self.xt0
                    self.dx0 = (xtr2 - self.xtr) * self.framestep / (self.params["FR2"] - iii)
                    self.dx = (- self.xtr - self.xtl - self.dx0 * ((self.params["FR2"] - iii) / self.framestep + 3)) / (
                            (self.params["FR2"] - iii) / self.framestep - 3)
                S = -self.params["AMPLITUDE"] / 2 * math.sin(self.alpha01 * iii)
                self.ytr = -S - self.d10
                self.ytl = -S + self.d10
                self.ztl = -self.params["GAIT_HEIGHT"]
                if iii == 0:
                    self.ztl = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] / 3
                elif iii == self.framestep:
                    self.ztl = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] * 2 / 3
                elif iii == self.params["FR2"] - self.framestep:
                    self.ztl = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] / 4
                elif iii == self.params["FR2"] - 2 * self.framestep:
                    self.ztl = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] * 2 / 4
                elif iii == self.params["FR2"] - 3 * self.framestep:
                    self.ztl = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"] * 3 / 4
                else:
                    self.ztl = -self.params["GAIT_HEIGHT"] + self.params["STEP_HEIGHT"]
                if self.cycle == self.number_of_cycles - 1:
                    if iii == (self.params["FR2"] - self.framestep):
                        self.ztl = -self.params["GAIT_HEIGHT"]
                        self.ytl = S + self.d10
                if iii == 0 or iii == self.framestep or iii == 2 * self.framestep:
                    self.xtl += self.dx0
                    self.ytl = S + self.d10 + self.dy0 * iii
                elif iii == self.params["FR2"] - self.framestep or iii == self.params["FR2"] - 2 * self.framestep or iii == self.params["FR2"] - 3 * self.framestep:
                    self.xtl += self.dx0
                    self.ytl = S + 64 + self.dy0 * 3 * self.framestep - self.dy * (self.params["FR2"] - self.hovernum * self.framestep) + self.dy0 * (
                            iii - (self.params["FR2"] - 3 * self.framestep))
                else:
                    self.xtl += self.dx
                    self.ytl = S + 64 + self.dy0 * 3 * self.framestep - self.dy * (iii - 3 * self.framestep)
                    self.wr = self.wr_target * (1 - iii / (self.params["FR2"] - self.hovernum * self.framestep) * 2)
                    self.wl = self.wl_target * (1 - iii / (self.params["FR2"] - self.hovernum * self.framestep) * 2)
                self.xtr += self.dx0
                self.ytr += self.dy0
                if self.ytl < 54:
                    self.ytl = 54

            new_positions = self.get_feet_action()

            self.walk_cycle_step += 1
            if self.walk_cycle_step == self.total_walk_cycle_steps:
                self.xr, self.xl, self.yr, self.yl = self.xr_old, self.xl_old, self.yr_old, self.yl_old
                self.general_step += 1
                self.walk_cycle_step = 0

        return new_positions


class BaseAgent(object):
    def __init__(self, random_mode=False, foot_only_mode=False):
        self.motion = MotionSim(random_mode=random_mode, foot_only_mode=foot_only_mode)
        self.prev_positions = None

    def act(self, state):
        positions = state[:-4]
        imu = state[-4:]
        if self.prev_positions is None:
            self.prev_positions = positions

        new_positions = self.motion.act(imu)
        if new_positions is not None and len(new_positions):
            self.prev_positions = new_positions

        return self.prev_positions
#         return new_positions


if __name__ == "__main__":
    print('This is not main module!')
