"""Install detector modules, ROS resources, and console entry points."""
from glob import glob
from setuptools import find_packages, setup
setup(
    name='dock_detector', version='0.1.0', packages=find_packages(include=['dock_detector', 'dock_detector.*']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/dock_detector']),
                ('share/dock_detector', ['package.xml', 'README.md']),
                ('share/dock_detector/config', glob('config/*.yaml')),
                ('share/dock_detector/launch', glob('launch/*.py'))],
    install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
    maintainer='Sergei Grichine', maintainer_email='slg@quakemap.com',
    description='Geometric LiDAR dock landmark detector for Nav2 docking', license='Apache-2.0',
    entry_points={'console_scripts': ['dock_detector = dock_detector.dock_detector_node:main',
                                        'dock_controller = dock_detector.docking_controller:main',
                                        'docking_alignment = dock_detector.docking_alignment:main',
                                        'generate_nav2_docking_config = dock_detector.nav2_config_merger:main']})
