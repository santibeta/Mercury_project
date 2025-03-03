from setuptools import find_packages, setup

package_name = 'spot_controller'

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
    maintainer='robotics',
    maintainer_email='santiagobetancourtcastillo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "udp_receiver = spot_controller.udp_receiver:main",
            "controller_tester = spot_controller.controller_tester:main",
            "odom_sender = spot_controller.odom_sender:main",
            "hololens_udp_relay = spot_controller.hololens_udp_relay:main",
            "hololens_service_cmd = spot_controller.hololens_service_cmd:main",
            "sit_stand_tester = spot_controller.sit_stand_tester:main"
        ],
    },
)

