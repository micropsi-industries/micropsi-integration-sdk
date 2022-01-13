import importlib
import importlib.util
import inspect
import logging
import os
from pathlib import Path

from micropsi_integration_sdk import robot_sdk

logger = logging.getLogger("system.micropsi_integration_sdk")


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
        logger.info("Searching %s for SDK robots.", filepath)
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
            # Ignore anything that's not a class definition
            if not isinstance(obj, type):
                continue

            # Ignore anything that's not an implementation of RobotInterface
            if not issubclass(obj, robot_sdk.RobotInterface):
                continue

            # Loudly ignore anything that's not fully implemented. This could indicate that some
            # abstract methods have been missed.
            if inspect.isabstract(obj):
                logger.info("%s is not fully implemented, skipping.", name)
                continue

            # If we got this far, we have a full implementation of a RobotInterface
            for robot_model in obj.get_supported_models():
                logger.info("Found SDK robot: %s", robot_model)
                self.__robots[robot_model] = obj

    def load_interface_directory(self, path):
        """
        Given a path to directory of files,
        attempt to load files
        """
        for f in os.listdir(path):
            if f == "__pycache__":
                continue
            abspath = os.path.join(path, f)
            try:
                self.load_interface(abspath)
            except Exception as e:
                logger.exception(e)
