from setuptools import find_packages, setup

package_name = 'hw01_solution'

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
    maintainer='HW1 student',
    maintainer_email='student@example.invalid',
    description='HW1 student package: camera-guided marker reaching.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_listener = hw01_solution.camera_listener:main',
            'marker_reacher = hw01_solution.marker_reacher:main',
        ],
    },
)
