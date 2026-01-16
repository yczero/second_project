from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_second_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zero',
    maintainer_email='yc4038@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'a_star_rpp = my_second_pkg.rpp_obstacle:main',
            'dwa = my_second_pkg.dwa:main',
            'hybrid_a_star = my_second_pkg.hybrid_a_star:main',
            'main_controller = my_second_pkg.main_controller:main',
            'rpp_obstacle = my_second_pkg.rpp_obstacle:main',
            
        ],
    },
)
