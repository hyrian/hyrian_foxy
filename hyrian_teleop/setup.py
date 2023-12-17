import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'hyrian_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='John woohyeon',
    author_email='jungwoohyeon@gmail.com',
    maintainer='John woohyeon',
    maintainer_email='jungwoohyeon@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: TO DO License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleoperation node using keyboard for the CrashLab - Hyrian'
    ),
    license='TO DO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = hyrian_teleop.script.teleop_keyboard:main',
            'teleop_joy = hyrian_teleop.script.teleop_joy:main' 
        ],
    },
)