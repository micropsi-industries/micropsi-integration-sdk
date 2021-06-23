from distutils.core import setup, Extension

setup(name='integration-sdk',
      version='0.1',
      packages=['micropsi_integration_sdk'],
      package_dir={'micropsi_integration_sdk': './micropsi_integration_sdk'},
      description='integration SDK for micropsi Industries',
      install_requires=['numpy'],
      )
