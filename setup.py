import os
from glob import glob

from setuptools import setup

package_name = 'cugo_ros2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.py')),
    ]
)
