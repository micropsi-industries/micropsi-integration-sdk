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
    RESULT_MESSAGES_V1,
    RESPONSE_MESSAGES_V1,
    RESULT_MESSAGES_V2,
    RESPONSE_MESSAGES_V2,
)
from micropsi_integration_sdk.robot_interface_collection import RobotInterfaceCollection
from micropsi_integration_sdk.robot_sdk import (
    RobotInterface,
    CartesianVelocityRobot,
    CartesianPoseRobot,
)

logger = logging.getLogger("server")


RESPONSE_MESSAGES = {
    1: RESPONSE_MESSAGES_V1,
    2: RESPONSE_MESSAGES_V2,
}


RESULT_MESSAGES = {
    1: RESULT_MESSAGES_V1,
    2: RESULT_MESSAGES_V2,
}


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
    parser.add_argument("--keepalive", type=float, default=60,
                        help="Keep idle client connections for KEEPALIVE seconds before dropping.")
    return parser.parse_args()


def main():
    logging.basicConfig(level=logging.INFO)
    args = parse_args()
    collection = RobotInterfaceCollection()
    collection.load_interface(args.robot_file)
    models = collection.list_robots()
    if len(models) == 0:
        raise RuntimeError(f"no robots found in {args.robot_file}")
    elif len(models) == 1:
        idx = 0
    else:
        for idx, model in enumerate(models):
            print(f"{idx}: {model}")
        idx = int(input(f"choose a model [0-{len(models) - 1}]: "))
    model = models[idx]
    robot_class = collection.get_robot_interface(model)
    robot = robot_class(ip_address=args.robot_address, model=model)
    with contextlib.closing(Server(address=args.server_address, robot=robot,
                                   always_fail=args.always_fail,
                                   keepalive=args.keepalive)):
        logger.info(f"mirai dev server listening on {args.server_address}. ctrl-c to interrupt.")
        while True:
            time.sleep(1)


class ClientDisconnected(Exception):
    pass


@contextlib.contextmanager
def final_shutdown(sock: socket.socket):
    try:
        yield
    finally:
        sock.shutdown(socket.SHUT_RDWR)


class Server(threading.Thread):
    def __init__(self, *args, robot: RobotInterface, address: str, always_fail: bool = False,
                 keepalive, **kwargs):
        super().__init__(*args, **kwargs, daemon=True)
        self.robot = robot
        self.address = address
        self.running = True
        self.executor = ThreadPoolExecutor(1)
        self.future = None
        self.result = Results.NoResult
        self.always_fail = always_fail
        self.keepalive = keepalive
        self.start()

    def close(self):
        self.running = False
        self.join()

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as main_socket:
            main_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            main_socket.settimeout(1)
            main_socket.bind((self.address, 6599))
            main_socket.listen(1)
            while self.running:
                try:
                    self.accept_client(main_socket)
                except socket.timeout:
                    continue

    def accept_client(self, sock: socket.socket):
        client, client_address = sock.accept()
        client.settimeout(1)
        with contextlib.closing(client):
            with final_shutdown(client):
                last_message_time = time.monotonic()
                while self.running:
                    try:
                        message = client.recv(1024)
                        response = self.handle_message(message)
                        logger.info("sending: %s", response)
                        client.sendall(response)
                    except socket.timeout:
                        if time.monotonic() - last_message_time > self.keepalive:
                            logger.info("keepalive time exceeded, dropping client")
                            break
                    except ClientDisconnected:
                        logger.info("client disconnected")
                        break
                    else:
                        last_message_time = time.monotonic()

    def handle_message(self, message: bytes) -> bytes:
        if message == b"":
            raise ClientDisconnected
        logger.info("received: %s", message)
        mark, api_version, message_type, message_bytes = unpack_header(message)
        response_messages = RESPONSE_MESSAGES[api_version]
        result_messages = RESULT_MESSAGES[api_version]
        if self.always_fail:
            response = response_messages[MessageType.FAILURE]
        elif message_type == MessageType.ExecuteSkill:
            self.result = Results.NoResult
            try:
                self.future = self.start_skill_execution()
            except Exception as e:
                logger.error(e)
                response = response_messages[MessageType.FAILURE]
            else:
                response = response_messages[MessageType.ExecuteSkill]
        elif message_type == MessageType.GetResult:
            if self.future is None or not self.future.done():
                response = result_messages[self.result]
            else:
                try:
                    self.result = self.future.result()
                except Exception as e:
                    logger.error(e)
                    response = response_messages[MessageType.FAILURE]
                else:
                    response = result_messages[self.result]
                finally:
                    self.future = None
        else:
            response = response_messages[message_type]
        return response

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
