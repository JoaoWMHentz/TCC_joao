from setuptools import setup

package_name = 'hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='João Witor Müller Hentz',
    maintainer_email='joao.hentz@catolicasc.edu.br',
    description='Driver ROS 2 da base diferencial (motores BLDC + IMU)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hardware_node = hardware.hardware_node:main',
        ],
    },
)
