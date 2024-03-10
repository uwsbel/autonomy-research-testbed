from setuptools import setup
import os

package_name = "localization_shared_utils"

# Automatically include all launch files.
launch_files = [(os.path.join('share', package_name, 'launch'), 
                 [os.path.join('launch', file) for file in os.listdir('launch')])]

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *launch_files,  # Add your launch files here
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="art",
    maintainer_email="art@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localization_shared_utils = localization_shared_utils.localization_shared_utils:main",
            'ins_publisher = localization_shared_utils.ins_publisher:main',
            'ins_gps_publisher = localization_shared_utils.ins_gps_publisher:main'
        ],
    },
)
