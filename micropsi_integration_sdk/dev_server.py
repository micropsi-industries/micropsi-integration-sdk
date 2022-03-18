#!/usr/bin/env python3
import argparse
import contextlib
import logging
import os
import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor, Future

import numpy as np

from micropsi_integration_sdk.dev_schema import (
    MessageType,
    Results,
    unpack_header,
    API_MARK,
    API_VERSION,
    RESULT_MESSAGES,
    RESPONSE_MESSAGES,
)
from micropsi_integration_sdk.robot_interface_collection import RobotInterfaceCollection
from micropsi_integration_sdk.robot_sdk import (
    RobotInterface,
    CartesianVelocityRobot,
    CartesianPoseRobot,
)

logger = logging.getLogger("server")


class ArgsFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawTextHelpFormatter):
    pass


def parse_args():
    parser = argparse.ArgumentParser(
        formatter_class=ArgsFormatter,
        epilog=os.linesep.join([
            "Usage example:",
            "# mirai-dev-server --robot-file examples/cartesian_velocity_robot.py"
        ]))
    parser.add_argument("--robot-file", required=True,
                        help="File where the sdk robot implementation can be loaded."
                             " eg: './examples/cartesian_velocity_robot.py'")
    parser.add_argument("--robot-address", default="localhost",
                        help="Address where the mirai dev server can expect to find the "
                             f"robot, for motion streaming.")
    parser.add_argument("--server-address", default="0.0.0.0",
                        help="Address that the mirai dev server should listen on.")
    parser.add_argument("--always-fail", action="store_true", default=False,
                        help="Cause the dev server to respond to every request with a failure"
                             " message.")
    return parser.parse_args()


def main():
    logging.basicConfig(level=logging.INFO)
    args = parse_args()
    robot_file = args.robot_file
    robot_address = args.robot_address
    server_address = args.server_address
    always_fail = args.always_fail
    collection = RobotInterfaceCollection()
    collection.load_interface(robot_file)
    models = collection.list_robots()
    if len(models) == 0:
        raise RuntimeError(f"no robots found in {robot_file}")
    elif len(models) == 1:
        idx = 0
    else:
        for idx, model in enumerate(models):
            print(f"{idx}: {model}")
        idx = int(input(f"choose a model [0-{len(models) - 1}]: "))
    model = models[idx]
    robot_class = collection.get_robot_interface(model)
    robot = robot_class(ip_address=robot_address, model=model)
    with contextlib.closing(Server(address=server_address, robot=robot, always_fail=always_fail)):
        logger.info(f"mirai dev server listening on {server_address}. ctrl-c to interrupt.")
        while True:
            time.sleep(1)


class ClientDisconnected(Exception):
    pass


class Server(threading.Thread):
    def __init__(self, *args, robot: RobotInterface, address: str, always_fail: bool = False,
                 **kwargs):
        super().__init__(*args, **kwargs, daemon=True)
        self.robot = robot
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(1)
        self.sock.bind((address, 6599))
        self.connection = None
        self.client_address = None
        self.running = True
        self.executor = ThreadPoolExecutor(1)
        self.future = None
        self.result = Results.NoResult
        self.always_fail = always_fail
        self.start()

    def __del__(self):
        self.sock.close()

    def close(self):
        self.running = False
        self.join()

    def run(self):
        with contextlib.closing(self.sock):
            self.sock.listen(1)
            while self.running:
                try:
                    self.accept_client()
                except socket.timeout:
                    continue

    def accept_client(self):
        self.connection, self.client_address = self.sock.accept()
        with contextlib.closing(self.connection):
            self.connection.settimeout(1)
            while self.running:
                try:
                    self.receive_data()
                except socket.timeout:
                    continue
                except ClientDisconnected:
                    logger.info("client disconnected")
                    break
        self.client_address = None

    def receive_data(self):
        message = self.connection.recv(1024)
        if message == b"":
            raise ClientDisconnected
        logger.info("received: %s", message)
        mark, api_version, message_type, message_bytes = unpack_header(message)
        if self.always_fail:
            response = RESPONSE_MESSAGES[MessageType.FAILURE]
        elif message_type == MessageType.ExecuteSkill:
            self.result = Results.NoResult
            try:
                self.future = self.start_skill_execution()
            except Exception as e:
                logger.error(e)
                response = RESPONSE_MESSAGES[MessageType.FAILURE]
            else:
                response = RESPONSE_MESSAGES[MessageType.ExecuteSkill]
        elif message_type == MessageType.GetResult:
            if self.future is None or not self.future.done():
                response = RESULT_MESSAGES[self.result]
            else:
                try:
                    self.result = self.future.result()
                except Exception as e:
                    logger.error(e)
                    response = RESPONSE_MESSAGES[MessageType.FAILURE]
                else:
                    response = RESULT_MESSAGES[self.result]
                finally:
                    self.future = None
        else:
            response = RESPONSE_MESSAGES[message_type]
        logger.info("sending: %s", response)
        self.connection.sendto(response, self.client_address)

    def start_skill_execution(self) -> Future:
        """
        Perform the initial connection handshake, and then defer the execution of a single skill
        cycle.
        """
        assert self.robot.connect() is True
        attempt = 0
        while not self.robot.is_ready_for_control():
            attempt += 1
            if attempt > 10:
                raise RuntimeError("robot never reported ready for control.")
            self.robot.prepare_for_control()
        self.robot.take_control()
        return self.executor.submit(self.execute_skill)

    def execute_skill(self):
        """
        This function roughly follows one skill execution cycle.
        It sends zeros for the velocity, so should be safe to use on real hardware, assuming the
        sdk class is correctly implemented.
        """
        step_count = 0
        frequency = self.robot.get_frequency()
        period = 1 / frequency
        for step in range(int(frequency) * 5):
            if step % int(frequency) == 0:
                logger.info("skill execution step %d", step)
            if not self.running:
                break
            step_count += 1
            start = time.perf_counter()
            state = self.robot.get_hardware_state()
            goal_pose = self.robot.forward_kinematics(joint_positions=state.joint_positions)
            assert self.robot.are_joint_positions_safe(joint_positions=state.joint_positions)
            if isinstance(self.robot, CartesianVelocityRobot):
                self.robot.send_velocity(velocity=np.random.randn(6) * 0.001, step_count=step_count)
            elif isinstance(self.robot, CartesianPoseRobot):
                self.robot.send_goal_pose(goal_pose=goal_pose, step_count=step_count)
            else:
                raise RuntimeError(f"unsupported robot type: {type(self.robot)}")
            elapsed = time.perf_counter() - start
            if elapsed < period:
                time.sleep(period - elapsed)
        logger.info("skill execution done")
        self.robot.release_control()
        self.robot.disconnect()
        return Results.Visual


if __name__ == "__main__":
    main()
