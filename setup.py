from pkg_resources import get_build_platform
from setuptools import setup
import os


def read(file_name):
    with open(os.path.join(os.path.dirname(__file__), file_name)) as handle:
        return handle.read()


setup(
    name="micropsi-integration-sdk",
    author="Micropsi Industries",
    author_email="contact@micropsi-industries.com",
    url="https://www.micropsi-industries.com/",
    description="Integration SDK for Micropsi Industries",
    long_description=read('README'),
    long_description_content_type='text/markdown',
    packages=["micropsi_integration_sdk"],
    install_requires=["numpy"],
    entry_points={
        "console_scripts": ["mirai-sandbox=micropsi_integration_sdk.sandbox:main"],
    },
    platforms=[get_build_platform()],
    python_requires=">=3.6, <4",
    license="MIT",
)
