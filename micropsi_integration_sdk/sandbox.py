from argparse import ArgumentParser

import time
import threading
from math import ceil
import numpy as np
import micropsi_integration_sdk.toolbox as tb
from micropsi_integration_sdk.robot_interface_collection import RobotInterfaceCollection


DEFAULT_IP = "192.168.100.100"
MAX_TCP_SPEED = 0.1
DEFAULT_TCP_SPEED = 0.05

DEFAULT_ACC = 1e-2
ACCURACY_MAX = 0.1

MAX_LINEAR_MOVEMENT = 0.1
DEF_FREQUENCY = 20

DEF_LENGTH = 0.05
MAX_LENGTH = 0.1

DEF_DIMENSION = 1

TIMEOUT = 2


def check_if_equal(arr_1, arr_2, E, pose=False, assrt=False):
    """
    Checks if two incomming poses are equivalent (delta < E).
    For pose only the linear component of position is verified
    """
    if pose:
        input_data = "TCP"
        arr_length = 3
    else:
        input_data = "Joint"
        arr_length = 6

    assert len(arr_1) == len(arr_2), "Invalid {} pose".format(input_data)

    try:
        assert np.all(np.abs(arr_1[:arr_length]-arr_2[:arr_length]) < E)
        return True

    except AssertionError as e:
        e.args += ("""Target {} pose not achieved.
        Expected: Pose {},
        Recieved: Pose {}.
        """.format(input_data, arr_1[:arr_length], arr_2[:arr_length]),)
        if assrt:
            raise e
        return False


class RobotCommunication(threading.Thread):
    """
    Connection thread to continuously fetch the robot state
    """
    def __init__(self, robot_interface, frequency):
        assert robot_interface is not None, "Invalid robot interface"
        self.rob = robot_interface
        self.frequency = frequency
        self.state = None
        self.running = True
        self.step = 0
        self.last_target = None

        threading.Thread.__init__(self)

        self.thread_error = None

        self.logs = []

        self.last_flush = 0

    def run(self):
        try:
            self.thread_stopped = False

            while self.running:
                start = time.time()

                self.state = self.get_state()

                elapsed = (time.time() - start)
                overstep = (1/self.frequency) - elapsed

                if overstep > 0:
                    time.sleep(overstep)
                else:
                    self.add_log("WARNING: Robot Frequency too high")

                secs_since_last_flush = time.time() - self.last_flush

                if secs_since_last_flush > 2:
                    self.flush_logs()
                    self.last_flush = time.time()

        except Exception as e:
            self.running = False
            self.thread_stopped = True
            self.debug_error = e
            raise

    def add_log(self, txt):
        if txt in self.logs:
            return
        self.logs.append(txt)

    def flush_logs(self):
        for i in self.logs:
            print(i)
        self.logs = []

    def move_joints(self, jnt_0, jnt_f, tcp_0, tcp_f, speed_lim_jnt,
                    speed_lim_tcp, tcp_accuracy, jnt_accuracy):
        """
        Moves the robot to target joint positions, by breaking down the delta
        into smaller deltas to sync with the frequency of robot communication.

        """
        assert jnt_f is not None
        assert jnt_0 is not None

        dist = tb.dist(tcp_0[:3], tcp_f[:3])

        if self.last_target is not None:
            jnt_0 = self.last_target.copy()

        jnt_diff = jnt_f - jnt_0

        vel = jnt_diff * self.frequency
        max_vel_ovrshoot = max(vel - speed_lim_jnt)
        delta_jnt = max(ceil(max_vel_ovrshoot), 1)

        delta = max(dist * self.frequency/speed_lim_tcp, delta_jnt)

        jnt_delta = jnt_diff/delta

        jnt_ = jnt_0.copy()
        for i in range(max(int(ceil(delta)), 1)):
            start = time.time()
            jnt_ = jnt_ + jnt_delta
            jnt_curr, tcp_curr = self.send_joint_positions(jnt_)
            elapsed = (time.time() - start)
            overstep = (1/self.frequency) - elapsed
            time.sleep(overstep)

        start = time.time()
        while not (check_if_equal(jnt_f, jnt_curr, jnt_accuracy) and
                   check_if_equal(tcp_f, tcp_curr, tcp_accuracy, pose=True)
                   ) and not self.thread_stopped:
            jnt_curr, tcp_curr = self.manual_step()
            jnt_curr, tcp_curr = self.send_joint_positions(jnt_)
            time.sleep(1/self.frequency)
            if time.time() - start > TIMEOUT:
                break

        check_if_equal(self.state.joint_positions, jnt_f, jnt_accuracy,
                       assrt=True)

    def move_robot(self, action, **kwargs):
        """
        Computes target joint values from actions and sends it to robot
        """

        jnt_acc = kwargs.get("jnt_accuracy")
        tcp_acc = kwargs.get("tcp_accuracy")
        jnt_0, tcp_0 = self.manual_step()
        jnt_0_1, tcp_0_1 = tb.get_modified_joints(self.rob, tcp_0,
                                                  jnt_0, trans=action)

        self.move_joints(jnt_0, jnt_0_1, tcp_0, tcp_0_1, **kwargs)

        jnt_1, tcp_1 = self.manual_step()

        check_if_equal(jnt_1, jnt_0_1, jnt_acc, assrt=True)
        check_if_equal(tcp_1, tcp_0_1, tcp_acc, pose=True, assrt=True)

        return jnt_0, jnt_0_1, tcp_1

    def get_state(self):
        """
        Connect to the robot and read the state.
        """
        state = None
        cnt = 0
        while state is None:
            cnt += 1
            state = self.rob.get_hardware_state()
            if state is None:
                if cnt > 10:
                    raise InterruptedError("Invalid state recieved, check"
                                           " robot connection")

                self.rob.clear_cached_hardware_state()
                self.rob.connect()
        return state

    def manual_step(self):
        jnt = self.state.joint_positions
        tcp = tb.extract_tcp(self.rob.forward_kinematics(jnt))
        self.step += 1
        return jnt, tcp

    def send_joint_positions(self, jnt):
        self.rob.send_joint_positions(jnt, self.frequency, self.step)
        self.last_target = np.array(jnt)
        return self.manual_step()

    def close(self):
        """
        Shutdowns Thread and close the connection to the robot
        """
        self.running = False
        self.join()

        # Release Control
        self.rob.release_control()
        self.rob.disconnect()
        print("{} Disconnected".format(self.rob.get_model()))


def parse_args():
    parser = ArgumentParser(description="Micropsi Industries Robot SDK Tool")
    parser._action_groups.pop()

    required = parser.add_argument_group("requried arguments")
    optional = parser.add_argument_group("optional arguments")

    required.add_argument("path", help="Path to the Robot implementation")
    required.add_argument("robot", nargs='?', default=None, help="Name of the"
                          " robot model as defined in the implementation")
    optional.add_argument("-f", "--frequency", default=DEF_FREQUENCY,
                          metavar='\b', type=float,
                          help="Frequency of the Robot. Default: {}Hz."
                          "".format(DEF_FREQUENCY))

    optional.add_argument("-s", "--tcp_speed", default=DEFAULT_TCP_SPEED,
                          type=float, metavar='\b', help=" TCP "
                          "speed in meter per second Default: {}, "
                          "Max: {}".format(DEFAULT_TCP_SPEED, MAX_TCP_SPEED))
    optional.add_argument("-d", "--dimension", default=DEF_DIMENSION, type=int,
                          metavar='\b', help="Number of Axis to move "
                          "the robot in. Default: {}".format(DEF_DIMENSION))

    optional.add_argument("-l", "--length", default=DEF_LENGTH, type=float,
                          metavar='\b', help="Length of movement, Default:{}"
                          " meters, Max: {}m".format(DEF_LENGTH, MAX_LENGTH))

    optional.add_argument("-ip", "--ip", default=DEFAULT_IP, metavar='\b',
                          help="IP address of the robot. Default:"
                          " {}".format(DEFAULT_IP))

    optional.add_argument("-j", "--joint_tolerance", default=DEFAULT_ACC,
                          type=float, metavar='\b', help="Accuracy of the "
                          "robot joints. Default: {} radians"
                          "".format(DEFAULT_ACC))

    optional.add_argument("-t", "--tcp_tolerance", default=DEFAULT_ACC,
                          type=float, metavar='\b',
                          help="Accuracy of the TCP position achieved by "
                          "robot. Default: {} meters".format(DEFAULT_ACC))
    optional.add_argument("-v", "--verbose", action="store_true",
                          help="(Flag) Enable traceback of failed tests.")
    return parser.parse_args()


def main():
    thread = None
    args = parse_args()
    debug = args.verbose
    path = args.path
    robot_model = args.robot
    robot_ip = args.ip

    robot_frequency = args.frequency
    dimensions = args.dimension if (args.dimension < 4 and args.dimension > 0) else 1
    dist = args.length if args.length <= MAX_LINEAR_MOVEMENT else MAX_LINEAR_MOVEMENT
    jnt_accuracy = args.joint_tolerance if args.joint_tolerance <= ACCURACY_MAX else ACCURACY_MAX
    tcp_accuracy = args.tcp_tolerance if args.tcp_tolerance <= ACCURACY_MAX else ACCURACY_MAX
    tcp_speed_lim = args.tcp_speed if args.tcp_speed <= MAX_TCP_SPEED else MAX_TCP_SPEED

    robot_path = tb.extract_path(path)

    collection = RobotInterfaceCollection()
    collection.load_interface_file(robot_path)
    supported_robots = sorted(collection.list_robots())

    try:
        if len(supported_robots) == 0:
            raise NotImplementedError("No Robot Implementation found.")

        if robot_model is None:
            if len(supported_robots) > 1:
                print("Multiple robot implmentations found.")
                print("Please enter the robot Model to use: "
                      "{}".format(supported_robots))
                robot_model = input("Model: ")
            else:
                print("Robot implementation found: "
                      "{}".format(supported_robots[0]))
                print("Loading {} in 2 seconds..".format(supported_robots[0]))
                time.sleep(2)
                robot_model = supported_robots[0]

        try:
            supported_robots.index(robot_model)
        except ValueError as e:
            raise NotImplementedError("Unknown/unsupported Robot model")

        robot_interface = collection.get_robot_interface(robot_model)

        robot_kwargs = {
            "frequency": robot_frequency,
            "model": robot_model,
            "ip_address": robot_ip,
        }

        guide_with_internal_ft = False
        if robot_interface.has_internal_ft_sensor():
            guide_with_internal_ft = True
            robot_kwargs["guide_with_internal_sensor"] = guide_with_internal_ft
        rob = robot_interface(**robot_kwargs)

        assert rob is not None, "Failed to load Robot implementation"

        thread = RobotCommunication(rob, robot_frequency)
        assert rob.get_model() is robot_model, "Invalid Robot model loaded"

        print("Robot {} implementation loaded".format(rob.get_model()))
        print("Connecting to Robot{}".format(robot_model))
        try:
            assert rob.connect(), "Robot connection failed"
        except Exception as e:
            err_txt = type(e).__name__
            raise ConnectionError("Robot connection failed. " + err_txt)

        thread.start()
        while thread.state is None and not thread.thread_stopped:
            time.sleep(0.1)
        print("Connected")
        jnt_speed_lim = rob.get_joint_speed_limits()
        assert thread.state is not None, "Invalid Robot State"

        jnt_cnt = rob.get_joint_count()
        jnt_speed_lmt = rob.get_joint_speed_limits()
        jnt_pos_lmt = rob.get_joint_position_limits()
        has_internal_ft = rob.has_internal_ft_sensor()

        if has_internal_ft and guide_with_internal_ft:
            assert len(thread.state.raw_wrench) == 6, "Invalid FT data"

        jnt_e = "Invalid joint positions"
        assert len(thread.state.joint_positions) == jnt_cnt, jnt_e
        assert len(jnt_pos_lmt) == jnt_cnt, jnt_e
        assert len(jnt_speed_lmt) == jnt_cnt, jnt_e

        rob.connect()

        rob.release_control()
        rob.prepare_for_control()

        jnt_0, tcp_0 = thread.manual_step()

        rob.release_control()
        rob.prepare_for_control()
        rob.take_control()

        kwargs = {
            "speed_lim_jnt": jnt_speed_lim,
            "speed_lim_tcp": tcp_speed_lim,
            "jnt_accuracy": jnt_accuracy,
            "tcp_accuracy": tcp_accuracy,
        }
        print("Moving in {} axes, with distance {}".format(dimensions, dist))

        # Send move to current position instruction to protect against
        # outdated move instructions in Robot register.
        thread.move_joints(jnt_0, jnt_0, tcp_0, tcp_0, **kwargs)
        check_if_equal(jnt_0, thread.state.joint_positions, jnt_accuracy,
                       assrt=True)

        actions = tb.gen_random_actions(dimensions, dist=dist)

        for i in range(len(actions)):
            print("Moving to: Position  {}".format(i + 1))
            j0, jf, tf = thread.move_robot(actions[i], **kwargs)
            time.sleep(2.1)

        check_if_equal(tcp_0, tf, tcp_accuracy, pose=True, assrt=True)
        check_if_equal(jnt_0, jf, jnt_accuracy, assrt=True)

    except Exception as e:
        e_list = []

        if thread and thread.debug_error is not None:
            e_list.insert(len(e_list), thread.debug_error)
        elif thread and thread.thread_stopped:
            err = RuntimeError("Robot communication failed")
            e_list.insert(len(e_list), err)

        e_list.insert(len(e_list), e)

        for err in e_list:
            if err is not None:
                if len(err.args) > 0:
                    s = err.args[0]
                else:
                    s = ""
                err_txt = type(err).__name__ + ": " + s
            print(err_txt)

        try:
            thread.close()
        except:
            pass

        if debug:
            raise
        exit()

    thread.close()


if __name__ == '__main__':
    main()
