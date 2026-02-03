from argparse import Namespace

import pytest

from micropsi_integration_sdk.sandbox import main as sandbox_main
from micropsi_integration_sdk.sandbox import DEFAULT_EE_SPEED_LINEAR, DEFAULT_EE_SPEED_ROT_DEGREES


@pytest.mark.parametrize(
    "path_model", [
        ("examples/cartesian_velocity_robot.py", "MyRobot Cartesian Velocity"),
        ("examples/cartesian_pose_robot.py", "MyRobot Cartesian Pose"),
        ("examples/joint_speed_robot.py", "MyRobot JointSpeed"),
        ("examples/joint_position_robot.py", "MyRobot JointPosition"),
    ]
)
def test_sandbox(path_model):
    path, model = path_model
    args = Namespace(
        path=path,
        model=model,
        ip_address="localhost",
        speed_linear=DEFAULT_EE_SPEED_LINEAR * 2,
        speed_angular=DEFAULT_EE_SPEED_ROT_DEGREES * 5,
    )
    sandbox_main(args)
