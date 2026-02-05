import logging

import numpy as np
from scipy.spatial.transform import Rotation

from micropsi_integration_sdk import CartesianPoseRobot, HardwareState

LOG = logging.getLogger(__name__)


class MyCartesianPoseRobot(CartesianPoseRobot):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.__connected = False
        self.__ready_for_control = False
        self.__controlled = False
        self.__pose = np.eye(4)

    @staticmethod
    def get_supported_models() -> list:
        return ["MyRobot Cartesian Pose"]

    def get_joint_count(self) -> int:
        return 6

    def get_joint_speed_limits(self) -> np.ndarray:
        return np.ones(6)

    def get_joint_position_limits(self) -> np.ndarray:
        return np.array([[-1, 1]] * 6)

    def connect(self) -> bool:
        self.__connected = True
        return True

    def disconnect(self) -> None:
        self.__connected = False

    def prepare_for_control(self) -> None:
        self.__ready_for_control = True

    def is_ready_for_control(self) -> bool:
        return self.__ready_for_control

    def take_control(self) -> None:
        self.__controlled = True

    def release_control(self) -> None:
        self.__controlled = False
        self.__ready_for_control = False

    def get_hardware_state(self) -> HardwareState:
        state = HardwareState(
            joint_positions=self.inverse_kinematics(end_effector_pose=self.__pose),
            end_effector_pose=np.copy(self.__pose),
        )
        LOG.debug("State: %s", state)
        return state

    def clear_cached_hardware_state(self) -> None:
        pass

    def forward_kinematics(self, *, joint_positions: np.ndarray) -> np.ndarray:
        # trivial mock kinematics
        pose = np.eye(4)
        pose[:3, 3] = joint_positions[:3]
        pose[:3, :3] = Rotation.from_rotvec(joint_positions[3:6]).as_matrix()
        return pose

    def inverse_kinematics(self, *, end_effector_pose: np.ndarray,
                           joint_reference=None) -> np.ndarray:
        # trivial mock kinematics
        position = end_effector_pose[:3, 3]
        axis_angle = Rotation.from_matrix(end_effector_pose[:3, :3]).as_rotvec()
        return np.concatenate([position, axis_angle])

    def send_goal_pose(self, *, goal_pose: np.ndarray, step_count: int) -> None:
        assert goal_pose.shape == (4, 4)
        self.__pose = np.copy(goal_pose)

    def are_joint_positions_safe(self, *, joint_positions: np.ndarray) -> bool:
        limits = self.get_joint_position_limits()
        for idx, position in enumerate(joint_positions):
            if not limits[idx][0] <= position <= limits[idx][1]:
                return False
        return True

    def command_move(self, *, joint_positions: np.ndarray) -> None:
        self.__pose = self.forward_kinematics(joint_positions=joint_positions)

    def command_stop(self) -> None:
        pass

    def get_slowdown_steps_in_seconds(self) -> float:
        return 0.05
