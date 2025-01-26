import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ghost_ilqr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
         (os.path.join('share', package_name), glob('ghost_ilqr/launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
        ('build/' + package_name, [package_name+'/ilqr.py']),
        ('build/' + package_name, [package_name+'/diff_drive_model.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='melcruz',
    maintainer_email='melissajecruz@utexas.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = ghost_ilqr.controller:main'
        ],
    },
)
