from setuptools import find_packages
from setuptools import setup


package_name = 'ackermann_command_guard'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ackermann SLAM maintainers',
    maintainer_email='maintainers@ackermann-slam.local',
    description=(
        'Feasibility and freshness guard for TwistStamped Ackermann commands.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_guard = ackermann_command_guard.command_guard:main',
        ],
    },
)
