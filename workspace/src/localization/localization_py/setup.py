from setuptools import setup
import os
from glob import glob

package_name = 'localization_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
<<<<<<< HEAD
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

=======
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
>>>>>>> 1de52e7dbc25b8c24491e906f4f65a06e8dacd32
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='art',
    maintainer_email='art@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_estimation = localization_py.state_estimation:main',
            'imu_publisher = localization_py.imu_publisher:main'
        ],
    },
)
