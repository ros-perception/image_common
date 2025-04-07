from setuptools import find_packages
from setuptools import setup

package_name = 'camera_info_manager_py'

setup(
    name=package_name,
    version='6.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author="Jack O'Quin",
    author_email='jack.oquin@gmail.com',
    maintainer='Jose Mastrangelo',
    maintainer_email='jmastrangelo@clearpathrobotics.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Python interface for camera calibration information.',
    long_description='Python interface for camera calibration information. \n\n'
                     'This ROS package provides a CameraInfo interface for Python camera\n'
                     'drivers similar to the C++ camera_info_manager package.',
    license='BSD',
    tests_require=['pytest'],
)
