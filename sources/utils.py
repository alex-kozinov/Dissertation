from pyrep import PyRep
from pyrep.objects import Dummy
from pyrep.robots.robot_component import RobotComponent


class StarkitRobot(RobotComponent):
    def __init__(self):
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
        self.initial_configuration = self.get_configuration_tree()