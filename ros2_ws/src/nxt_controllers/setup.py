from setuptools import setup

package_name = 'nxt_controllers'

setup(
    name=package_name,
    version='5.0.0',
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
    description='Python-based controllers for the BetaBot diff drive base and joints.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'base_controller = nxt_controllers.base_controller:main',
            'base_odometry = nxt_controllers.base_odometry:main',
            'joints_controller = nxt_controllers.joints_controller:main',
            'joint_states_aggregator = nxt_controllers.joint_states_aggregator:main',
        ],
    },
)
