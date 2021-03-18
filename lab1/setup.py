from setuptools import setup
import os
from glob import glob

package_name = 'lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kamil',
    maintainer_email='kamil.szczepanik00@gmail.com',
    description='Creating custom turtle_teleop',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_teleop = lab1.my_pub:main',
            'keyboard_reader_node = lab1.keyboard_reader:main'
        ],
    },
)
