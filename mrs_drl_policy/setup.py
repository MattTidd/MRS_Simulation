from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mrs_drl_policy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # need to include the DRL models:
        *[
            (
                os.path.join("share", package_name, os.path.dirname(f)),
                [f]
            )
            for f in glob('policy/**/*', recursive = True)
            if os.path.isfile(f)
        ]
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matthew',
    maintainer_email='mtidd2@unb.ca',
    description='Package for performing single-agent DRL navigation for use in MRS',
    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            "policy_node = mrs_drl_policy.policy_node:main" ,
            "goal_client = mrs_drl_policy.goal_client.main"
        ],
    },
)
