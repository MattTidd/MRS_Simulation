from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mrs_bt_handler'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/trees', glob('trees/*.py')),
        ('share/' + package_name + '/suitability_model', glob('suitability_model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matthew',
    maintainer_email='mtidd2@unb.ca',
    description='This package holds the main BT functionality for an agent in the MRS',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "bt_node = mrs_bt_handler.bt_node:main"
        ],
    },
)
