import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'limo_gazebosim'

setup(
    name=package_name,
    version='0.0.1', # Adjust version as needed
    packages=find_packages(exclude=['test']), # Finds Python modules if you add any
    data_files=[
        # Install marker file
        ('share/ament_index/resource_index/packages',
            ['package.xml']),
        # Install package name marker file (needed for resource discovery)
        (os.path.join('share', package_name), ['package.xml']), # Installs package.xml into share folder as well
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.*')), # Include .world and potentially others
        # Install config files (including the YAML bridge config)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install models - This requires installing contents of subdirectories explicitly
        # Install files directly in models/limo/
        (os.path.join('share', package_name, 'models', 'limo'), glob('models/limo/*.*')), # Catches .sdf, .config
        # Install files in models/limo/meshes/
        (os.path.join('share', package_name, 'models', 'limo', 'meshes'), glob('models/limo/meshes/*.*')), # Catches .dae, .stl, .png etc.
        # Install files directly in models/test_zone/
        (os.path.join('share', package_name, 'models', 'test_zone'), glob('models/test_zone/*.*')),
        # Install files in models/test_zone/meshes/
        (os.path.join('share', package_name, 'models', 'test_zone', 'meshes'), glob('models/test_zone/meshes/*.*')),
        # Add more lines here if you have other model subdirectories
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name', # Replace with your name
    maintainer_email='your.email@example.com', # Replace with your email
    description='Gazebo simulation package for LIMO robot', # Replace/improve description
    license='Apache License 2.0', # Or your chosen license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any Python executables here if you create them later
            # 'my_node = limo_gazebosim.my_node:main',
        ],
    },
)