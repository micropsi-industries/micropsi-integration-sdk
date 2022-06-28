import pytest

from micropsi_integration_sdk.sandbox import main
from argparse import Namespace


@pytest.mark.parametrize("path_model", [
    ("examples/cartesian_velocity_robot.py", "MyRobot Cartesian Velocity"),
    ("examples/cartesian_pose_robot.py", "MyRobot Cartesian Pose"),
    ("examples/joint_speed_robot.py", "MyRobot JointSpeed"),
    ("examples/joint_position_robot.py", "MyRobot JointPosition"),
])
def test_main(path_model):
    path, model = path_model
    args = Namespace(
        verbose=False,
        path=path,
        model=model,
        ip_address="localhost",
        length=.01,
        tolerance_linear=.001,
        tolerance_angular=.001,
        speed_linear=.05,
        speed_angular=.05,
        dimension=3,
    )
    main(args)
