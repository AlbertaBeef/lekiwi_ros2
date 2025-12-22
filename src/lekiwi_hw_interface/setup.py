from setuptools import find_packages, setup

package_name = 'lekiwi_hw_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*", 'lerobot', 'lerobot.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AlbertaBeef',
    maintainer_email='grouby177@gmail.com',
    description='LeKiwi HW interface (motor bridge).',
    license='Apache License 2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lekiwi_motor_bridge = lekiwi_hw_interface.lekiwi_motor_bridge:main',
        ],
    },
)
