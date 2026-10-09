from glob import glob
from setuptools import setup

setup(
    name='obstacle_zone_tools', version='0.2.0',
    packages=['obstacle_zone_tools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/obstacle_zone_tools']),
        ('share/obstacle_zone_tools', ['package.xml', 'README.md', 'LICENSE']),
        ('share/obstacle_zone_tools/launch', glob('launch/*.launch.py')),
        ('share/obstacle_zone_tools/config', glob('config/*.yaml')),
        ('share/obstacle_zone_tools/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='obstacle_zone_tools contributors', maintainer_email='maintainer@example.com',
    description='RViz rectangle zones controlling Nav2 obstacle layer', license='Apache-2.0',
    entry_points={'console_scripts': [
        'obstacle_zone_editor = obstacle_zone_tools.node:edit_main',
        'obstacle_zone_runner = obstacle_zone_tools.node:run_main',
    ]},
)
