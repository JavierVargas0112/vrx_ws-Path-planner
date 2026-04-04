from setuptools import setup

package_name = 'gps_waypoints'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='you',
    description='Nodo que publica checkpoints GPS desde JSON',
    entry_points={
        'console_scripts': [
            'gps_waypoint_node = gps_waypoints.waypoint_node:main',
            'gps_waypoint_converter = gps_waypoints.convert_node:main'
            ,
            'gps_waypoint_controller = gps_waypoints.controller_node:main'
        ],
    },
)
