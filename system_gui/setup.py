from setuptools import find_packages, setup

package_name = 'system_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matthew',
    maintainer_email='mtidd2@unb.ca',
    description='Contains the GUI node used for debugging and interfacing with the MRS',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            "system_gui_node = system_gui.system_gui:main",
            "bt_gui_node = system_gui.bt_gui:main"
        ],
    },
)
