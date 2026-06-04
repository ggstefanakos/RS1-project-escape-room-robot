from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'escape_room_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Το αλλάζουμε για να βλέπει το νέο αρχείο
        (os.path.join('share', package_name, 'launch'), ['launch/robot_brain.launch.py'])  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='stefanakosgeorg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
