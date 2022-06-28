import os.path
import sys


def pytest_cmdline_main(config):
    root_dir = os.path.dirname(__file__)
    examples_dir = os.path.join(root_dir, "examples")
    sys.path.append(examples_dir)
