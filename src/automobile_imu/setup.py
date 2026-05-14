from setuptools import find_packages, setup

package_name = 'automobile_imu'

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
    maintainer='Jayamal Sigera (UniPd DriveOps)',
    maintainer_email='jayamalsigeras@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bno055_imu_node = automobile_imu.bno055_imu_node:main',
        ],
    },
)
