import os
from glob import glob
from setuptools import setup

package_name = 'neuronbot2_led'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ting Chang',
    maintainer_email='ting.chang@adlinktech.com',
    description='NeuronBot2 LED',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'led_control = neuronbot2_led.led_control:main'
        ],
    },
)
