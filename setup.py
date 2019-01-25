from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['networkx_ros'],
    package_dir={'networkx_ros': 'src/networkx_ros'}
)

setup(**d)
