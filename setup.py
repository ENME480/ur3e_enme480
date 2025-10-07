from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur3e_enme480'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaustubh',
    maintainer_email='enme480@umd.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'ur3e_sim_enme480_ctrl = ur3e_enme480.ur3e_sim_enme480_ctrl:main',
        	'ur3e_sim_enme480_topics = ur3e_enme480.ur3e_sim_enme480_topics:main',
            'ur3e_fk = ur3e_enme480.ur3e_fk:main',
        ],
    },
)
