from setuptools import find_packages, setup

package_name = 'turtlesim_exploration'

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
    maintainer='cielo',
    maintainer_email='nicolosicielo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "obstacle_manager_node = turtlesim_exploration.obstacle_manager_node:main",
            "explorer_node = turtlesim_exploration.explorer_node:main"
        ],
    },
)