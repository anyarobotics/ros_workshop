from setuptools import find_packages, setup

package_name = 'amr_handler'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/handler.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tatwik',
    maintainer_email='yenikapatitatwik@gmail.com',
    description='Differential drive and odometry nodes for the AMR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_node = amr_handler.diff_drive_node:main',
            'odometry_node = amr_handler.odometry_node:main',
        ],
    },
)
