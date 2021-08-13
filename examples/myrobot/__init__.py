from typing import Optional

import numpy as np

from micropsi_integration_sdk import JointPositionRobot, HardwareState

SUPPORTED_MODELS = {
    "MyRobot Arm": {
        "joint_count": 6,
        "joint_speed_limits": np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]),
        "joint_position_limits": np.array([[-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi],
                                           [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi]])
    },
    "MyRobot Gantry": {
        "joint_count": 4,
        "joint_speed_limits": np.array([.2, .2, .2, np.pi]),
        "joint_position_limits": np.array([[-.5, .5], [-.5, .5], [-.5, .5], [-np.pi, np.pi]])
    }
}


class MyRobot(JointPositionRobot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__connected = False
        self.__ready_for_control = False
        self.__controlled = False

    @staticmethod
    def get_supported_models() -> list:
        return list(SUPPORTED_MODELS.keys())

    def get_joint_count(self) -> int:
        return SUPPORTED_MODELS[self.model]["joint_count"]

    def get_joint_speed_limits(self) -> np.array:
        return SUPPORTED_MODELS[self.model]["joint_speed_limits"]

    def get_joint_position_limits(self) -> np.array:
        return SUPPORTED_MODELS[self.model]["joint_position_limits"]

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

    def get_hardware_state(self) -> Optional[HardwareState]:
        pass

    def clear_cached_hardware_state(self) -> None:
        pass

    def forward_kinematics(self, *, joint_positions: np.array) -> np.array:
        pass

    def inverse_kinematics(self, *, end_effector_pose: np.array,
                           joint_reference: Optional[np.array]) -> Optional[np.array]:
        pass

    def are_joint_positions_safe(self, *, joint_positions: np.array) -> bool:
        return False

    def send_joint_positions(self, *, joint_positions: np.array, step_count: int) -> None:
        pass

    def command_move(self, *, joint_positions: np.array) -> None:
        pass

    def command_stop(self) -> None:
        pass
