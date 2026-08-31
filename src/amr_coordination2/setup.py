from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_coordination2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shimsha_22',
    maintainer_email='shimsha_22@todo.todo',
    description='Decentralized AMR Coordination Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr_node = amr_coordination2.amr_node:main'
        ],
    },
)

