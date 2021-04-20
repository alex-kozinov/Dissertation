#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json
from compute_Alpha_v3 import Alpha

class Glob:
    def __init__(self, current_work_directory):
        self.current_work_directory = current_work_directory
        with open(current_work_directory + "Soccer/Init_params/Sim/Sim_params.json", "r") as f:
            self.params = json.loads(f.read())

class Motion1:
    def __init__(self, glob):
        self.glob = glob
        self.params = self.glob.params
        self.ACTIVESERVOS = [(10,2),(9,2),(8,2),(7,2),(6,2),(5,2),(4,2),
                (3,2),(2,2),(1,2),(0,2),(10,1),(9,1),(8,1),
                (7,1),(6,1),(5,1),(4,1),(3,1),(2,1),(1,1)]
        # (10,2) Прав Стопа Вбок, (9,2) Прав Стопа Вперед,(8,2) Прав Голень, (7,2) Прав Колено,
        # (6,2)  Прав Бедро,      (5,2) Прав Таз,         (1,2) Прав Ключица,(4,2) Прав Локоть, (0,2) Торс
        # (10,1) Лев Стопа Вбок,  (9,1) Лев Стопа Вперед, (8,1) Лев Голень,  (7,1) Лев Колено
        # (6,1)  Лев Бедро,       (5,1) Лев Таз,          (1,1) Лев Ключица, (4,1) Лев Локоть

        self.servo_Trims = [0 for i in range(len(self.ACTIVESERVOS))]

        #FACTOR =  [ 1,-1,-1,1,-1,-1, 1,1,1,-1,1,-1,-1, 1,1,-1,-1, 1,1,1,-1,-1, 1]  # v2.3
        self.FACTOR =  [ 1,1,1,-1,1,1, 1,1,1,1,1,1,1, 1,-1,1,1, 1,1,1,1, 1, 1]  # Surrogat 1
        a5 = 21.5  # мм расстояние от оси симметрии до оси сервы 5
        b5 = 18.5  # мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
        c5 = 0     # мм расстояние от оси сервы 6 до нуля Z по вертикали
        a6 = 42    # мм расстояние от оси сервы 6 до оси сервы 7
        a7 = 65.5  # мм расстояние от оси сервы 7 до оси сервы 8
        a8 = 63.8  # мм расстояние от оси сервы 8 до оси сервы 9
        a9 = 35.5  # мм расстояние от оси сервы 9 до оси сервы 10
        a10= 25.4  # мм расстояние от оси сервы 10 до центра стопы по горизонтали
        b10= 26.4  # мм расстояние от оси сервы 10 до низа стопы   26.4
        c10 = 12   # мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
        self.SIZES = [ a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 ]
        self.d10 = 53.4 #53.4 # расстояние по Y от центра стопы до оси робота
        limAlpha5 = [-2667, 2667]
        limAlpha6 = [-3000,  740]
        limAlpha7 = [-3555, 3260]
        limAlpha8 = [-4150, 1777]
        limAlpha9 = [-4000, 2960]
        limAlpha10 =[-2815,   600]
        LIMALPHA = [limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10]
        self.MOTION_SLOT_DICT = {0:['',0], 1:['',0], 2:['',0], 3:['',0], 4:['',0], 5:['Get_Up_Inentification',7000],
                    6:['Soccer_Get_UP_Stomach_N', 5000], 7:['Soccer_Get_UP_Face_Up_N', 5000], 8:['Soccer_Walk_FF',0], 9:['',0], 10:['',0],
                    11:['',0], 12:['',0], 13:['',0], 14:['Soccer_Small_Jump_Forward',0], 15:['',0],
                    16:['',0], 17:['',0], 18:['Soccer_Kick_Forward_Right_Leg',5000], 19: ['Soccer_Kick_Forward_Left_Leg',5000], 20:['',0],
                    21:['Get_Up_From_Defence',1000], 22:['',0], 23:['PanaltyDefenceReady_Fast',500], 24:['PenaltyDefenceF',300], 25:['',0],
                    26:['',0], 27:['',0], 28:['',0], 29:['',0], 30:['Soccer_Walk_FF0',0],
                    31:['Soccer_Walk_FF1',0], 32:['Soccer_Walk_FF2',0], 33: ['Soccer_Get_UP_Stomach',0], 34:['Soccer_Get_UP_Face_Up',0],
                    35: ['Get_Up_Right',0], 36: ['PenaltyDefenceR',2000], 37: ['PenaltyDefenceL',2000]}
        self.stepLengthPlaner_is_ON = False
        self.TIK2RAD = 0.00058909
        self.slowTime   = 0.0             # seconds
        self.simThreadCycleInMs = 20
        self.frame_delay = self.glob.params['FRAME_DELAY']
        self.frames_per_cycle = self.glob.params['FRAMES_PER_CYCLE']
        self.stepLength = 0.0    # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
        self.sideLength = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
        self.rotation = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.
        self.first_Leg_Is_Right_Leg = False
        # Following paramenetrs Not recommended for change
        self.amplitude = 32          # mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
        self.fr1 =8                  # frame number for 1-st phase of gait ( two legs on floor)
        self.fr2 = 12                # frame number for 2-nd phase of gait ( one leg in air)
        self.gaitHeight= 190         # Distance between Center of mass and floor in walk pose
        self.stepHeight = 32.0       # elevation of sole over floor
        self.initPoses = 400//self.simThreadCycleInMs
        self.limAlpha1 =LIMALPHA
        self.limAlpha1[3][1]=0
        #  end of  paramenetrs Not recommended for change
        self.al = Alpha()
        self.exitFlag = 0
        self.falling_Flag = 0
        self.neck_pan = 0
        self.old_neck_pan = 0
        self.body_euler_angle ={}
        self.old_neck_tilt = 0
        self.direction_To_Attack = 0
        self.activePose = []
        self.xtr = 0
        self.ytr = -self.d10   #-53.4
        self.ztr = -self.gaitHeight
        self.xr = 0
        self.yr = 0
        self.zr = -1
        self.wr = 0
        self.xtl = 0
        self.ytl = self.d10   # 53.4
        self.ztl = -self.gaitHeight
        self.xl = 0
        self.yl = 0
        self.zl = -1
        self.wl = 0
        self.euler_angle = {}
        self.modified_roll = 0
        self.robot_In_0_Pose = False
        self.tempangle = 0
        self.tempangles =[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.ACTIVEJOINTS = ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5','hand_right_4',
            'hand_right_3','hand_right_2','hand_right_1','Tors1','Leg_left_10','Leg_left_9','Leg_left_8',
            'Leg_left_7','Leg_left_6','Leg_left_5','hand_left_4','hand_left_3','hand_left_2','hand_left_1','head0','head12']

    #-------------------------------------------------------------------------------------------------------------------------------
    def imu_body_yaw(self):
        yaw = self.neck_pan*self.TIK2RAD + self.euler_angle['yaw']
        yaw = self.norm_yaw(yaw)
        return yaw

    def norm_yaw(self, yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:  yaw -= 2* math.pi
        if yaw < -math.pi: yaw += 2* math.pi
        return yaw

    def quaternion_to_euler_angle(self, quaternion):
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

    def push_Button(self, button):
        pressed_button = button.wait_for_button_pressing()
        print("нажато")
        return pressed_button
        #ala = 0
        #while(ala==0):
        #    if (self.pin2.value()== 0):   # нажатие на кнопку 2 на голове
        #        ala = 1
        #        print("нажато")
        #self.pyb.delay(1000)

    def play_Motion_Slot(self, name = ''):
        self.simulateMotion(name = name)

    def play_Soft_Motion_Slot(self, name = ''):             # the slot from file will be played in robot
        self.simulateMotion(name = name)

    def falling_Test(self):
        if self.pause_key_is_pressed:
            #self.lock.acquire()
            self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
            self.pause_key_is_pressed = False
            while (True):
                if self.pause_key_is_pressed:
                    #self.lock.release()
                    self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                    self.pause_key_is_pressed = False
                    break

        if self.stop_key_is_pressed:
            print('Simulation STOP by keyboard')
            self.sim_Stop()
            time.sleep(0.1)
            self.sim_Disable()
            sys.exit(0)

        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
        if (self.body_euler_angle['pitch']) < -0.785:
            self.falling_Flag = 1                   # on stomach
            self.simulateMotion(name = 'Soccer_Get_UP_Stomach_N')
        if (self.body_euler_angle['pitch']) >  0.785:
            self.falling_Flag = -1                  # face up
            self.simulateMotion(name = 'Soccer_Get_UP_Face_Up')
        if 0.785 < (self.body_euler_angle['roll']) < 2.356:
            self.falling_Flag = -2                  # on right side
            self.simulateMotion(name = 'Get_Up_Right')
        if -2.356 < (self.body_euler_angle['roll']) < -0.785:
            self.falling_Flag = 2                   # on left side
            self.simulateMotion(name = 'Get_Up_Left')
        return self.falling_Flag

    def computeAlphaForWalk(self,sizes, limAlpha, hands_on = True ):
        angles =[]
        anglesR=[]
        anglesL=[]
        anglesR = self.al.compute_Alpha_v3(self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, sizes, limAlpha)
        anglesL = self.al.compute_Alpha_v3(self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,self.wl, sizes, limAlpha)

        if len(anglesR)>1:
            for i in range(len(anglesR)):
                if len(anglesR)==1: break
                if anglesR[0][2]<anglesR[1][2]: anglesR.pop(1)
                else: anglesR.pop(0)
        elif len(anglesR)==0:
            return[]
        if len(anglesL)>1:
            for i in range(len(anglesL)):
                if len(anglesL)==1: break
                if anglesL[0][2]<anglesL[1][2]: anglesL.pop(1)
                else: anglesL.pop(0)
        elif len(anglesL)==0:
            return[]
        if self.first_Leg_Is_Right_Leg == True:
            for j in range(6): angles.append(anglesR[0][j])
            if hands_on: angles.append(1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.524 - self.xtl/57.3)
            else: angles.append(0.0)
            angles.append(0.0)
            #for j in range(5): angles.append(0.0)
            for j in range(6): angles.append(-anglesL[0][j])
            #for j in range(4): angles.append(0.0)
            if hands_on: angles.append(-1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.524 + self.xtr/57.3)
            else: angles.append(0.0)
        else:
            for j in range(6): angles.append(anglesL[0][j])
            if hands_on: angles.append(1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.524 - self.xtr/57.3)
            else: angles.append(0.0)
            angles.append(0.0)                                  # Tors
            for j in range(6): angles.append(-anglesR[0][j])
            if hands_on: angles.append(-1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.524 + self.xtl/57.3)
            else: angles.append(0.0)
        self.activePose = angles
        return angles

    def activation(self):
        time.sleep(0.1)
        self.sim.simxStartSimulation(self.clientID,self.sim.simx_opmode_oneshot)
        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
        returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
        self.direction_To_Attack += self.euler_angle['yaw']
        #print('body_euler_angle = ', self.body_euler_angle)
        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        self.Dummy_HData.append(Dummy_Hposition)
        #self.Dummy_1_YawData.append(self.body_euler_angle[0])
        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        self.direction_To_Attack = self.norm_yaw(self.direction_To_Attack)

    def reOrderServoData(self, servoDatas):
        order = [0, 11, 1, 12, 2, 13, 3, 14, 4, 15, 5, 16, 6, 17, 7, 18, 8, 19, 9, 20, 10]
        servoDatasOrdered = []
        for orderNumber in order:
            servoDatasOrdered.append(servoDatas[orderNumber])
        return servoDatasOrdered

    def walk_Initial_Pose(self):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            if self.falling_Flag == 3: print('STOP!')
            else: print('FALLING!!!', self.falling_Flag)
            return[]
        self.xtr = self.xtl = 0
        amplitude = 70
        framestep = self.simThreadCycleInMs//10
        for j in range (self.initPoses):
            self.ztr = -223.1 + j*(223.1-self.gaitHeight)/self.initPoses
            self.ztl = -223.1 + j*(223.1-self.gaitHeight)/self.initPoses
            self.ytr = -self.d10 - j*amplitude/2 /self.initPoses
            self.ytl =  self.d10 - j*amplitude/2 /self.initPoses
            angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                self.wait_sim_step()
                self.sim.simxPauseCommunication(self.clientID, True)
                for i in range(len(angles)):
                    returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                self.sim.simx_opmode_oneshot)
                self.sim.simxPauseCommunication(self.clientID, False)

                time.sleep(self.slowTime)
                returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                      self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                #self.Dummy_HData.append(Dummy_Hposition)
                #self.timeElapsed = self.timeElapsed +1
                # print(Dummy_Hposition)

    def walk_cycle(self, stepLength, sideLength, rotation,cycle, number_Of_Cycles, secondStepLength = 1000):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            if self.falling_Flag == 3: print('STOP!')
            else: print('FALLING!!!', self.falling_Flag)
            return[]
        self.stepLength = stepLength
        self.sideLength = sideLength
        self.rotation = math.degrees(rotation)
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if rotation>0 or sideLength<0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        rotation = -self.rotation/286
        alpha01 =  math.pi/self.fr2
        frameNumberPerCycle = 2*self.fr2
        frameNumberPerStep = self.fr2
        framestep = self.simThreadCycleInMs//10
        hovernum = 6     # number of steps hovering over take off + landing points
        xt0 = self.stepLength /2 * self.fr2 / (self.fr2 + framestep * hovernum)
        xtr0 = - self.stepLength /2 * self.fr2 / (self.fr2 + framestep * hovernum)
        dx0_typical = self.stepLength/(self.fr2+ hovernum * framestep)*framestep        # CoM propulsion forward per framestep
        dy0 = self.sideLength / (self.fr2 + hovernum * framestep) * framestep        # CoM propulsion sideways per framestep
        dy = self.sideLength /(self.fr2 - hovernum * framestep) * framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        wr_old = self.wr
        wl_old = self.wl
        wr_target = - rotation
        wl_target = - rotation
                                                          # FASA 1 (Left support leg)
        if self.stepLengthPlaner_is_ON: print('pitch:', self.body_euler_angle['pitch'])
        xt0, dy0, dy = self.stepLengthPlaner(self.stepLength, self.sideLength, framestep, hovernum)
        self.ztl = -self.gaitHeight
        xtl0  = self.xtl
        xtr0  = self.xtr
        xtl1 = -xt0
        xtr1 = xt0
        dx0 = (xtl1 - xtl0) * framestep / self.fr2
        dx = (xtr1 - xtr0) * framestep / self.fr2 * (self.fr2 + hovernum * framestep)/ (self.fr2 - hovernum * framestep)
        for iii in range(0, frameNumberPerStep, framestep):
            start1 = 0
            if 2 * framestep < iii <  self.fr2 - 4 * framestep:
                xt0, dy0, dy = self.stepLengthPlaner(self.stepLength, self.sideLength, framestep, hovernum)
                #if cycle == 3 and iii >= 2 * framestep:
                #    xt0, dy0, dy = self.stepLengthPlaner(self.stepLength, 60, framestep, hovernum)
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
                #print('dx01=', dx01, 'dx0=', dx0, )
                #print('stepLength1 =', stepLength1)
            S = self.amplitude/2 *math.sin(alpha01 * iii )
            self.ytr = -S - self.d10
            self.ytl = -S + self.d10
            self.ztr = -self.gaitHeight
            if iii == self.fr2 - 4 * framestep and self.stepLengthPlaner_is_ON:
                #print('self.modified_roll =', self.modified_roll)
                if self.modified_roll > 0.1 or self.body_euler_angle['pitch'] > 0.1:
                    for p in range(10):
                        #print('time:', self.timeElapsed, 'pause right')
                        self.sim_Progress(0.01)
                        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
                        if self.body_euler_angle['roll'] > 0:
                            self.modified_roll = self.body_euler_angle['roll'] - math.pi
                        else: self.modified_roll = self.body_euler_angle['roll'] + math.pi
                        if self.modified_roll < 0.1 or self.body_euler_angle['pitch'] < 0.1: break
            if iii== 0 :
                self.ztr = -self.gaitHeight + self.stepHeight / 3
            elif iii== framestep:
                self.ztr = -self.gaitHeight + self.stepHeight * 2 / 3
            elif iii==self.fr2 - framestep:
                self.ztr = -self.gaitHeight + self.stepHeight / 4
            elif iii==self.fr2 - 2 * framestep:
                self.ztr = -self.gaitHeight + self.stepHeight * 2 / 4
            elif iii==self.fr2 - 3 * framestep:
                self.ztr = -self.gaitHeight + self.stepHeight * 3 / 4
            else:
                self.ztr = -self.gaitHeight + self.stepHeight
            if iii==0 or iii== framestep or iii== 2 * framestep:
                #self.xtr = xtr0 + dx0 * (iii / framestep + 1)
                self.xtr += dx0
                self.ytr = -64 + dy0 * iii
            elif iii==self.fr2 - framestep or iii== self.fr2 - 2 * framestep or iii== self.fr2 - 3 * framestep:
                #self.xtr = xtr0 + dx * (self.fr2 / framestep - hovernum) +  dx0 * ( iii - self.fr2 + 7 * framestep)/ framestep
                self.xtr += dx0
                self.ytr = -64 + dy0 * 3 * framestep  - dy*(self.fr2 - 3 * framestep)/2 + dy0 * (iii - (self.fr2 - 3 * framestep))
            else:
                #self.xtr = xtr0 + dx * (iii/framestep - 2) + dx0 * 3
                self.xtr += dx
                self.ytr = - 64 + dy0 * 3 * framestep - dy*iii/2
                self.wr = wr_old + (wr_target - wr_old) * (iii)/(self.fr2 - hovernum * framestep)
                self.wl = wl_old + (wl_target - wl_old) * (iii)/(self.fr2- hovernum * framestep)
            #self.xtl = xtl0 + dx0 * (iii/framestep + 1)
            self.xtl += dx0
            self.ytl = -S + self.d10 + dy0 * iii
            #if iii > self.fr2 / 3 * 2:
            #    self.xl = 3 * self.xtl/ self.gaitHeight
            #else:
            #    self.xl = 0
            successCode = self.feet_Action(start1)
            #print(iii, self.xtr, self.ytr, self.ztr, self.xr, self.yr, self.zr, self.wr, self.xtl, self.ytl, self.ztl, self.xl, self.yl, self.zl, self.wl, successCode)

                                        # FASA 2 ( Right support leg)

        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        if self.stepLengthPlaner_is_ON: print('pitch:', self.body_euler_angle['pitch'])
        xt0, dy0, dy = self.stepLengthPlaner(secondStepLength, self.sideLength, framestep, hovernum)
        xtl1  = self.xtl
        xtr1  = self.xtr
        #xtl1 = -xt0
        #xtr1 = xt0
        self.ztr = -self.gaitHeight
        if cycle == number_Of_Cycles - 1:
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
                xt0, dy0, dy = self.stepLengthPlaner(secondStepLength, self.sideLength, framestep, hovernum)
                xtl1 = -xt0
                xtr1 = xt0
                if cycle == number_Of_Cycles - 1:
                    xtl2 = 0
                    xtr2 = 0
                else:
                    xtl2 = xt0
                    xtr2 = -xt0
                dx0 = (xtr2 - self.xtr) * framestep / (self.fr2 - iii)
                dx = (- self.xtr - self.xtl - dx0  * ( (self.fr2 - iii)/ framestep + 3)) /( (self.fr2 - iii)/ framestep - 3)
                #print('dx0=', dx0, 'dx=', dx)
                #print('stepLength1 =', stepLength1)
            S = -self.amplitude/2 *math.sin(alpha01 * iii)
            self.ytr = -S - self.d10
            self.ytl = -S + self.d10
            self.ztl = -self.gaitHeight
            if iii == self.fr2 - 4 * framestep and self.stepLengthPlaner_is_ON:
                #print('self.modified_roll =', self.modified_roll)
                if self.modified_roll < 0 or self.body_euler_angle['pitch'] > 0.1:
                    for p in range(10):
                        #print('time:', self.timeElapsed, 'pause left')
                        self.sim_Progress(0.01)
                        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
                        if self.body_euler_angle['roll'] > 0:
                            self.modified_roll = self.body_euler_angle['roll'] - math.pi
                        else: self.modified_roll = self.body_euler_angle['roll'] + math.pi
                        if self.modified_roll > 0.0 or self.body_euler_angle['pitch'] < 0.1: break
            if iii == 0:
                self.ztl = -self.gaitHeight + self.stepHeight / 3
            elif iii == framestep:
                self.ztl = -self.gaitHeight + self.stepHeight * 2 / 3
            elif iii == self.fr2 - framestep:
                self.ztl = -self.gaitHeight + self.stepHeight / 4
            elif iii == self.fr2 - 2 * framestep:
                self.ztl = -self.gaitHeight + self.stepHeight * 2 / 4
            elif iii == self.fr2 - 3 * framestep:
                self.ztl = -self.gaitHeight + self.stepHeight * 3 / 4
            else:
                self.ztl = -self.gaitHeight + self.stepHeight
            if cycle == number_Of_Cycles - 1:
                if iii== (self.fr2 - framestep):
                    self.ztl = -self.gaitHeight
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
                self.wr = wr_target * (1 - (iii)/(self.fr2- hovernum * framestep) * 2)
                self.wl = wl_target * (1 - (iii)/(self.fr2- hovernum * framestep) * 2)
            self.xtr += dx0
            self.ytr += dy0
            if self.ytl < 54 : self.ytl = 54
            #if iii > self.fr2 / 3 * 2:
            #    self.xr = 3 * self.xtr/ self.gaitHeight
            #else:
            #    self.xr = 0
            successCode = self.feet_Action(start1)
            #print(iii + self.fr2, self.xtr, self.ytr, self.ztr, self.xr, self.yr, self.zr, self.wr, self.xtl, self.ytl, self.ztl, self.xl, self.yl, self.zl, self.wl, successCode)
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        #self.first_Leg_Is_Right_Leg = tmp1

    def stepLengthPlaner(self, regularStepLength, regularSideLength, framestep, hovernum):
        if self.stepLengthPlaner_is_ON == True:
            stepLength1 = regularStepLength
            if - 0.08 <= self.body_euler_angle['pitch'] < - 0.045:
                #stepLength1 += (-self.body_euler_angle['pitch']) * 2000
                if stepLength1 > 140: stepLength1 = 140
                if stepLength1 < -140: stepLength1 = -140
                print('stepLength1=', stepLength1)
            elif self.body_euler_angle['pitch'] < - 0.1:
                stepLength1 = 140
                print('stepLength1=', stepLength1)
            elif 0.045 < self.body_euler_angle['pitch'] <= 0.1:
                #stepLength1 += (-self.body_euler_angle['pitch']) * 2000
                if stepLength1 > 140: stepLength1 = 140
                if stepLength1 < -140: stepLength1 = -140
                print('stepLength1=', stepLength1)
            elif 0.08 < self.body_euler_angle['pitch'] :
                stepLength1 = -140
                print('stepLength1=', stepLength1)
            xt0 = stepLength1 /2 * self.fr2 / (self.fr2 + framestep * hovernum)
            if self.body_euler_angle['roll'] > 0:
                self.modified_roll = self.body_euler_angle['roll'] - math.pi
            else: self.modified_roll = self.body_euler_angle['roll'] + math.pi
            if self.modified_roll > 0.2 : 
                sideLength = 60
                print('sideLength = ', sideLength)
            if self.modified_roll < -0.2 : 
                sideLength = -60
                print('sideLength = ', sideLength)
            else: sideLength = regularSideLength
            dy0 = sideLength / (self.fr2 + hovernum * framestep) * framestep        # CoM propulsion sideways per framestep
            dy = sideLength /(self.fr2 - hovernum * framestep) * framestep
        else:
            xt0 = regularStepLength /2 * self.fr2 / (self.fr2 + framestep * hovernum)
            dy0 = regularSideLength / (self.fr2 + hovernum * framestep) * framestep        # CoM propulsion sideways per framestep
            dy = regularSideLength /(self.fr2 - hovernum * framestep) * framestep
        return xt0, dy0, dy

    def feet_Action(self, start1):
        angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
        if not self.falling_Flag ==0: return
        if len(angles)==0:
            self.exitFlag = self.exitFlag +1
            return False
        else:
            self.wait_sim_step()
            self.sim.simxPauseCommunication(self.clientID, True)
            for i in range(len(angles)):
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                            self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                            self.sim.simx_opmode_oneshot)
            self.sim.simxPauseCommunication(self.clientID, False)

            time.sleep(self.slowTime)
            returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
            self.Dummy_HData.append(Dummy_Hposition)
            returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
            self.BallData.append(self.Ballposition)
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
            self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
            self.Dummy_1_YawData.append(self.imu_body_yaw())
            self.Dummy_1_PitchData.append(self.body_euler_angle['pitch'])
            if self.body_euler_angle['roll'] > 0:
                self.modified_roll = self.body_euler_angle['roll'] - math.pi
            else: self.modified_roll = self.body_euler_angle['roll'] + math.pi
            self.Dummy_1_RollData.append(self.modified_roll)
            #print(self.euler_angle)
            self.timeElapsed = self.timeElapsed +1
            delta_angles = []
            for an in range(21):
                delta_angles.append(round((angles[an] - self.tempangles[an])/0.2, 2))
            #print(delta_angles)
            report2 = ''
            position_o = int(self.gaitHeight * math.tan(self.modified_roll))
            position_l = self.ytl
            position_r = self.ytr
            self.position_o.append(position_o)
            if self.ztl == -self.gaitHeight:
                self.position_l.append(position_l)
            else: self.position_l.append(-1000)
            if self.ztr == -self.gaitHeight:
                self.position_r.append(position_r)
            else: self.position_r.append(1000)
            self.tempangles = angles
            return True

    def walk_Final_Pose(self):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            if self.falling_Flag == 3: print('STOP!')
            else: print('FALLING!!!', self.falling_Flag)
            return[]
        initPoses = self.initPoses * 4
        framestep = self.simThreadCycleInMs//10
        for j in range (initPoses):
            self.ztr = -self.gaitHeight - (j+1)*(223.1-self.gaitHeight)/initPoses
            self.ztl = -self.gaitHeight - (j+1)*(223.1-self.gaitHeight)/initPoses
            self.ytr = -self.d10 - (initPoses-(j+1))*self.amplitude/2 /initPoses
            self.ytl =  self.d10 - (initPoses-(j+1))*self.amplitude/2 /initPoses
            if j == initPoses - 1:
                angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1, hands_on = False)
            else: angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
                print('walk_Final_Pose: failure with IK')
            else:
                self.wait_sim_step()
                self.sim.simxPauseCommunication(self.clientID, True)
                for i in range(len(angles)):
                    returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                 self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                 self.sim.simx_opmode_oneshot)
                self.sim.simxPauseCommunication(self.clientID, False)

                time.sleep(self.slowTime)
                returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                      self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)

    def refresh_Orientation(self):
        returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
        self.euler_angle['yaw'] -= self.direction_To_Attack

    def normalize_rotation(self, yaw):
        if abs(yaw) > 2 * math.pi: yaw %= (2 * math.pi)
        if yaw > math.pi : yaw -= (2 * math.pi)
        if yaw < -math.pi : yaw += (2 * math.pi)
        if yaw > 0.5 : yaw = 0.5
        if yaw < -0.5 : yaw = -0.5
        return yaw

if __name__=="__main__":
    print('This is not main module!')


