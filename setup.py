from distutils.core import setup, Extension

setup(name='mirai-sandbox',
      version='0.1',
      packages=['micropsi_integration_sdk'],
      description='integration SDK for micropsi Industries',
      install_requires=['numpy'],
      entry_points = { 
            "console_scripts" : ['mirai-sandbox=micropsi_integration_sdk.sandbox:main'],
      },
      )
