from setuptools import find_packages, setup

package_name = 'ekf_gps_imu'

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
    maintainer='rian',
    maintainer_email='rian@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_node = ekf_gps_imu.ekf_node:main',
            'csv_publisher = ekf_gps_imu.csv_publisher:main',
        ],
    },
)
