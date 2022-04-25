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

    def register_interface(self, interface_class: type) -> bool:
        """
        Given a class definition fully satisfying the RobotInterface ABC, register it as the 
        constructor for each of its supported robot models.
        Args:
            interface_class (type): class definition, fully implementing RobotInterface.

        Returns:
            bool: True if registered, False otherwise.
        """
        # Ignore anything that's not a class definition
        if not isinstance(interface_class, type):
            return False

        # Ignore anything that's not an implementation of RobotInterface
        if not issubclass(interface_class, robot_sdk.RobotInterface):
            return False

        # Loudly ignore anything that's not fully implemented. This could indicate that some
        # abstract methods have been missed.
        if inspect.isabstract(interface_class):
            logger.info("%s is not fully implemented, skipping.", interface_class.__qualname__)
            return False

        # If we got this far, we have a full implementation of a RobotInterface
        for robot_model in interface_class.get_supported_models():
            logger.info("Found SDK robot: %s", robot_model)
            self.__robots[robot_model] = interface_class
        return True

    def unregister_interface(self, interface_class: type):
        """
        Remove any robot models from the collection which are registered against the provided
        interface class.
        """
        self.__robots = {k: v for k, v in self.__robots.items() if v is not interface_class}

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
        for _, obj in inspect.getmembers(module):
            self.register_interface(obj)

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
