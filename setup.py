from setuptools import find_packages, setup

package_name = 'behavior_tree_pkg'

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
    maintainer='urc',
    maintainer_email='jddent@mix.wvu.edu',
    description='This package is a behavior tree for a rover',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = behavior_tree_pkg.rover_control:main',
            'control_subscriber = behavior_tree_pkg.rover_control_sub:main',
            'behavior_tree = behavior_tree_pkg.behavior_tree:main'
        ],
    },
)
