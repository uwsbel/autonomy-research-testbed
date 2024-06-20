from setuptools import find_packages, setup

package_name = 'imu_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bret',
    maintainer_email='bretwitt@hawaii.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": ["imu_filter = imu_filter.imu_filter:main",
                            "tcp_pose = imu_filter.tcp_pose:main"
        ],
    },
)
