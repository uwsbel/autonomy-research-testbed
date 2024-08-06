from setuptools import setup

package_name = 'convoy_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='art',
    maintainer_email='bretwitt@hawaii.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ "follower = convoy_controller.follower:main",
                             "leader = convoy_controller.leader:main",
                             "mpc = convoy_controller.mpc:main",
                             "velocity = convoy_controller.velocity:main",
                             "control_mux = convoy_controller.control_mux:main",
                             "vehicle_traj = convoy_controller.vehicle_trajectory:main"],
                           
    },
)
