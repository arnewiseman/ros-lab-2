from setuptools import find_packages, setup

package_name = 'py_wall_attack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/wall_follower.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wisemaa3',
    maintainer_email='wisemaa3@wwu.edu',
    # ← give it something informative
    description='Wall‐following node for WWU CSCI 497F/597F Lab 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = py_wall_attack.publisher_member_function:main',
        ],
    },
)
