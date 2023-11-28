from setuptools import find_packages, setup

package_name = 'tank_control_pkg'

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
    maintainer='jeffh',
    maintainer_email='jeffh@opensar.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tank_control = tank_control_pkg.tank_control:main",
            "linear_test = tank_control_pkg.linear_movement_test:main",
        ],
    },
)
