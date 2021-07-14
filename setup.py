from distutils.core import setup, Extension

setup(name='integration-sdk',
      version='0.1',
      packages=['micropsi_integration_sdk'],
      description='integration SDK for micropsi Industries',
      install_requires=['numpy'],
      entry_points = { 
            "console_scripts" : ['ri=micropsi_integration_sdk.ri_tool:main'],
      },
      )
