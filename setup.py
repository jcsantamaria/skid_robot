## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    # packages=['skid_robot'],
    # package_dir={'': 'src'},
    # install_requires=['Adafruit-PCA9685']
    packages=['roboclaw_driver'],
    package_dir={'': 'src'},
)

setup(**setup_args)
