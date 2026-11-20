from setuptools import find_packages, setup

package_name = 'hw02_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'matplotlib'],
    zip_safe=True,
    maintainer='HW02 student',
    maintainer_email='student@example.invalid',
    description='HW02 student map inputs and path planning',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner_inputs = hw02_solution.planner_inputs:main',
        ],
    },
)
