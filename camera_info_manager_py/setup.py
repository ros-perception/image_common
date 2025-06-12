import os

from setuptools import setup

package_name = 'camera_info_manager_py'

setup(
    name=package_name,
    version='5.1.7',
    packages=['camera_info_manager'],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose Mastrangelo',
    maintainer_email='jmastrangelo@clearpathrobotics.com',
    author="Jack O'Quin",
    author_email='jack.oquin@gmail.com',
    description='Python interface for camera calibration information.',
    long_description='Python interface for camera calibration information. \n\n'
                     'This ROS package provides a CameraInfo interface for Python camera\n'
                     'drivers similar to the C++ camera_info_manager package.',
    license='BSD',
    tests_require=['pytest'],
)
