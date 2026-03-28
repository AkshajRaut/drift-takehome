from setuptools import setup

package_name = 'tidybot_navigation'

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
    maintainer='User',
    maintainer_email='user@example.com',
    description='Autonomous navigation and pick-and-place for TidyBot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigator      = tidybot_navigation.navigator:main',
            'arm_controller = tidybot_navigation.arm_controller:main',
            'gripper        = tidybot_navigation.gripper:main',
            'mapper         = tidybot_navigation.mapper:main',
        ],
    },
)
