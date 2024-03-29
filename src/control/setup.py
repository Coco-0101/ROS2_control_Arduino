from setuptools import setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='betty',
    maintainer_email='a1095105@mail.nuk.edu.tw',
    description='Control Arduino in ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "find_ports = control.find_ports:main",
            "local_controller = control.local_controller:main",
            "remote_controller = control.remote_controller:main",
            "arduino_subscriber = control.arduino_subscriber:main",
            "keyboard = control.keyboard:main",
        ],
    },
)
