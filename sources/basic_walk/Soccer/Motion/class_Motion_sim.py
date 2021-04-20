#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if sys.version != '3.4.0':
    current_work_directory += '/'
    with open("simulator_lib_directory.txt", "r") as f:
        simulator_lib_directory = f.read()
    simulator_lib_directory = simulator_lib_directory.replace('\\', '/')
    sys.path.append(simulator_lib_directory)
    import random
    import sim, threading
else:
    import starkit
    sys.path.append('/')

sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')


from class_Motion import *
from class_Motion import MotionBase
from compute_Alpha_v3 import Alpha


class MotionSim(MotionBase):
    def __init__(self, glob):
        self.yaw_timer = time.perf_counter()
        self.FRAMELENGTH = 0.02
        import random as random
        self.random = random
        import sim as vr
        self.sim = vr
        import numpy as np
        self.np = np
        import matplotlib.pyplot as plt
        self.plt = plt
        import keyboard as keyboard
        self.keyboard = keyboard
        self.pause_key_is_pressed = False
        self.stop_key_is_pressed = False
        self.keyboard.on_press_key('p', self.on_pause)
        self.keyboard.on_press_key('s', self.on_stop)
        self.Dummy_HData =[]
        self.Dummy_1_YawData = []
        self.Dummy_1_PitchData = []
        self.Dummy_1_RollData = []
        self.BallData =[]
        self.timeElapsed = 0
        self.trims = []
        self.jointHandle = []
        self.Dummy_HHandle = 0
        self.Dummy_1Handle = 0
        self.BallHandle = 0
        self.Ballposition = []
        self.position_o =[]
        self.position_l = []
        self.position_r = []
        self.clientID = -1
        self.sim_step_counter = 0
        super().__init__(glob)

    def on_pause(self, e):
        self.pause_key_is_pressed = True

    def on_stop(self, e):
        self.stop_key_is_pressed = True

    def wait_sim_step(self):
        while True:
            self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_buffer)
            tim = self.sim.simxGetLastCmdTime(self.clientID)
            #print ('Simulation time: ', tim)
            if tim > self.sim_step_counter:
                self.sim_step_counter = tim
                break
            time.sleep(0.001)
            if self.stop_key_is_pressed:
                self.sim_Stop()
                time.sleep(0.1)
                self.sim_Disable()
                sys.exit(0)

    def simulateMotion(self, number = 0, name = ''):
        #mot = [(0,'Initial_Pose'),(1,0),(2,0),(3,0),(4,0),(5,'Get_Up_Left'),
        #   (6,'Soccer_Get_UP_Stomach_N'),(7,0),(8,'Soccer_Walk_FF'),(9,0),(10,0),
        #   (11,0),(12,0),(13,0),(14,'Soccer_Small_Jump_Forward'),(15,0),
        #   (16,0),(17,0),(18,'Soccer_Kick_Forward_Right_Leg'),(19,'Soccer_Kick_Forward_Left_Leg'),(20,0),
        #   (21,'Get_Up_From_Defence'),(22,0),(23,'PanaltyDefenceReady_Fast'), (24,'PenaltyDefenceF'),(25,0),
        #   (26,0),(27,0),(28,0),(29.0),(30,'Soccer_Walk_FF0'),
        #   (31,'Soccer_Walk_FF1'), (32,'Soccer_Walk_FF2'), (33,'Soccer_Get_UP_Stomach'), (34,'Soccer_Get_UP_Face_Up'),
        #   (35,'Get_Up_Right'), (36,'PenaltyDefenceR'), (37,'PenaltyDefenceL')]
        # start the simulation
        if number > 0 and name == '': name = self.MOTION_SLOT_DICT[number]
        with open(current_work_directory + "Soccer/Motion/motion_slots/" + name + ".json", "r") as f:
            slots = json.loads(f.read())
        mot_list = slots[name]
        i=0
        for i in range(len(mot_list)):
            if  self.falling_Flag ==3: return
            activePoseOld = []
            for ind in range(len(self.activePose)): activePoseOld.append(self.activePose[ind])
            self.activePose =[]
            for j in range(len(self.ACTIVEJOINTS) - 2):
                    self.activePose.append(0.017*mot_list[i][j+1]*0.03375)
            pulseNum = int(mot_list[i][0]*self.FRAMELENGTH * 1000 / self.simThreadCycleInMs)
            for k in range (pulseNum):
                self.sim.simxPauseCommunication(self.clientID, True)
                for j in range(len(self.ACTIVEJOINTS) - 2):
                    tempActivePose = activePoseOld[j]+(self.activePose[j]-activePoseOld[j])*k/pulseNum
                    returnCode = self.sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                                 tempActivePose*self.FACTOR[j] +self.trims[j], self.sim.simx_opmode_streaming)
                self.sim.simxPauseCommunication(self.clientID, False)
                self.sim.simxSynchronousTrigger(self.clientID)
        return

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
    def sim_start(self):
        self.sim.simxFinish(-1) # just in case, close all opened connections
        simThreadCycleInMs = 5
        self.clientID=self.sim.simxStart('127.0.0.1', -19997, True, True, 5000, simThreadCycleInMs) # Connect to V-REP
            ## Collect Joint Handles and trims from model
        returnCode, self.Dummy_HHandle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy_H', self.sim.simx_opmode_blocking)
        returnCode, self.Dummy_1Handle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy1' , self.sim.simx_opmode_blocking)
        returnCode, self.BallHandle = self.sim.simxGetObjectHandle(self.clientID, 'Ball', self.sim.simx_opmode_blocking)
        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_streaming)
        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
        returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
        returnCode, Dummy_1position= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
        for i in range(len(self.ACTIVEJOINTS)):
            returnCode, handle= self.sim.simxGetObjectHandle(self.clientID, self.ACTIVEJOINTS[i], self.sim.simx_opmode_blocking)
            self.jointHandle.append(handle)
            returnCode, position= self.sim.simxGetJointPosition(self.clientID, handle, self.sim.simx_opmode_blocking)
            self.trims.append(position)
            self.activePose.append(position)
        self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_streaming)


    def sim_Stop(self):
        self.sim.simxStopSimulation(self.clientID,self.sim.simx_opmode_oneshot)
                # return to initial pose
        for j in range(len(self.ACTIVEJOINTS)):
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[j] , self.trims[j], self.sim.simx_opmode_oneshot)


    def sim_Disable(self):            # Now close the connection to Sim:
        time.sleep(0.2)
        self.sim.simxFinish(self.clientID)


if __name__=="__main__":
    print('This is not main module!')


