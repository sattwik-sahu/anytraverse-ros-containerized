from glob import glob

from setuptools import setup

package_name = 'hound_base'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karthiknambiar29',
    maintainer_email='karthik23@iiserb.ac.in',
    description='Hound differential-drive base: /cmd_vel to Sabertooth motor controllers.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'base_driver = hound_base.base_driver:main',
            'teleop_wasd = hound_base.teleop_wasd:main',
            'calibrate_base = hound_base.calibrate_base:main',
        ],
    },
)
