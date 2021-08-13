import importlib
import importlib.util
import inspect
import os
from pathlib import Path

from micropsi_integration_sdk import JointPositionRobot


class RobotInterfaceCollection:
    def __init__(self):
        self.__robots = {}

    def list_robots(self):
        return list(self.__robots.keys())

    def get_robot_interface(self, robot_model):
        return self.__robots[robot_model]

    def load_interface_file(self, filepath):
        """
        Given a path to a file implementing a robot class
        inheriting from the ServoRobot, store this class
        in the __robots dict.
        """
        module_id = str(uuid.uuid4())
        spec = importlib.util.spec_from_file_location(name=module_id, location=filepath)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for name, obj in inspect.getmembers(module):
            if not isinstance(obj, type):
                continue
            if (issubclass(obj, JointPositionRobot)
                    and obj is not JointPositionRobot):
                for robot_model in obj.get_supported_models():
                    self.__robots[robot_model] = obj

    def load_interface_directory(self, path, errors=[]):
        """
        Given a path to directory of files,
        attempt to load files
        """
        for f in os.listdir(path):
            if f.endswith(".py"):
                abspath = os.path.join(path, f)
                try:
                    err = self.load_interface_file(abspath)
                    if err:
                        errors.extend(err)
                except Exception as e:
                    errors.extend(e)

        return errors