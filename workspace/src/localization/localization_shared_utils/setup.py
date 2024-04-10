from setuptools import setup
import os

package_name = 'localization_shared_utils'


launch_directory = os.path.join('launch')
launch_files = [
    os.path.join(launch_directory, file)
    for file in os.listdir(launch_directory)
    if file.endswith('.py')  
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
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
            'localization_shared_utils = localization_shared_utils.localization_shared_utils:main'
        ],
    },
)

