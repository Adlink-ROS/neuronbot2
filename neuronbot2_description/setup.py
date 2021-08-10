import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = 'neuronbot2_description'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    # Packages to export
    packages=[PACKAGE_NAME],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        # Include our package.xml file
        (os.path.join('share', PACKAGE_NAME), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='My awesome package.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'spawn_nb2 = neuronbot2_description.spawn_nt2:main',
        ],
    },
)