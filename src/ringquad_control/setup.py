from glob import glob

from setuptools import find_packages, setup

package_name = 'ringquad_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rajeet',
    maintainer_email='rajeet@tautomatic.ai',
    description='Flight and ground controller for the morphing ring-propeller quad.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = ringquad_control.node:main',
        ],
    },
)
