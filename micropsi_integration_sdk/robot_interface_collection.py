import importlib
import importlib.util
import inspect
import logging
import os
from pathlib import Path

from micropsi_integration_sdk import robot_sdk

logger = logging.getLogger(__name__)


class RobotInterfaceCollection:
    def __init__(self):
        self.__robots = {}

    def list_robots(self):
        return list(self.__robots.keys())

    def get_robot_interface(self, robot_model):
        return self.__robots[robot_model]

    def load_interface(self, filepath):
        """
        Given a path to a python module implementing a non-abstract robot class inheriting from the
        RobotInterface, store this class in the __robots dict against any model names it claims to
        support.
        """
        filepath = Path(filepath)
        module_id = filepath.name
        while '.' in module_id:
            module_id = os.path.splitext(module_id)[0]
        module_id = module_id
        if filepath.is_dir():
            spec = importlib.util.spec_from_file_location(
                name=module_id, location=str(filepath / '__init__.py'),
                submodule_search_locations=[str(filepath)])
        else:
            spec = importlib.util.spec_from_file_location(name=module_id, location=str(filepath))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for name, obj in inspect.getmembers(module):
            if not isinstance(obj, type):
                continue
            if issubclass(obj, robot_sdk.RobotInterface) and not inspect.isabstract(obj):
                for robot_model in obj.get_supported_models():
                    self.__robots[robot_model] = obj

    def load_interface_directory(self, path):
        """
        Given a path to directory of files,
        attempt to load files
        """
        for f in os.listdir(path):
            abspath = os.path.join(path, f)
            try:
                self.load_interface(abspath)
            except Exception as e:
                logger.exception(e)
