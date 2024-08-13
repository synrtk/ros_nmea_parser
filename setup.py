from glob import glob
import os
from setuptools import setup

PACKAGE_NAME = "ros_nmea_parser"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='2.0.1',
    packages=["libros_nmea_parser", "libros_nmea_parser.nodes"],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml")))],
    package_dir={'': 'src', },
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyserial',
                      'numpy',
                      'pyyaml'],
    author='whyoo',
    maintainer='whyoo',
    keywords=['ROS2'],
    description='Package to parse NMEA strings and publish a very simple GPS message.',
    license='BSD',
    entry_points={
        'console_scripts': ['nmea_subscriber = libros_nmea_parser.nodes.nmea_subscriber:main',
                            'nmea_publisher = libros_nmea_parser.nodes.nmea_publisher:main'],
    }
)
