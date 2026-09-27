from setuptools import setup

package_name = 'nxt2_ros'

setup(
    name=package_name,
    version='5.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodolphe Matias de Sousa',
    maintainer_email='rodolphe.matias@icloud.com',
    description='Bindings between the NXT2 brick and ROS 2.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'nxt2_ros = nxt2_ros.nxt2_ros:main',
        ],
    },
)
