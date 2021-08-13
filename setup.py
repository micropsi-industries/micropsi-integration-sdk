from distutils.core import setup

setup(
      name="micropsi-integration-sdk",
      packages=["micropsi_integration_sdk"],
      description="Integration SDK for Micropsi Industries.",
      install_requires=["numpy"],
      entry_points={
            "console_scripts": ["mirai-sandbox=micropsi_integration_sdk.sandbox:main"],
      }
)
