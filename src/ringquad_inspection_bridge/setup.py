from setuptools import find_packages, setup

package_name = 'ringquad_inspection_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rajeet',
    maintainer_email='rajeet@tautomatic.ai',
    description='Streams simulated camera frames and telemetry to the host-side scouting harness.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_bridge = ringquad_inspection_bridge.camera_bridge:main',
        ],
    },
)
