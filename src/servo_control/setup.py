from setuptools import setup

package_name = 'servo_control'

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
    description='Package for read and publish serial communication data to ROS2 system.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
	    'console_scripts': [
            "find_ports = servo_control.find_ports:main",
            "servo = servo_control.servo:main",
	    ],
    },
)
