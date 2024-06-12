from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['deformable_simulator_scene_utilities'],
    package_dir={'': 'src'}
)

setup(**setup_args)