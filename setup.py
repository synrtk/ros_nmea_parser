from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['libros_nmea_parser', 'libros_nmea_parser.nodes'],
    package_dir={'': 'src'}
)

setup(**d)
