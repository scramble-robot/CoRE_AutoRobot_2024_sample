import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'corejp_sample_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CoRE competition committee',
    maintainer_email='info-core@scramble-robot.org',
    description='sample program for CoRE 2024 auto robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stop_sign_detector = corejp_sample_detector.stop_sign_detector:main',
        ],
    },
)
