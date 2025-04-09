from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'gtec_mmwave_reader'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'example_radar_configs'), glob('example_radar_configs/*.cfg')),
    ],
    zip_safe=True,
    maintainer='Valentin Barral',
    maintainer_email='valentin.barral@udc.es',
    description='The gtec_mmwave_reader package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'out_of_box_reader = gtec_mmwave_reader.IWR6843ISK.IWR6843ISK_out_of_box_reader:main',
            'people_counting_reader = gtec_mmwave_reader.IWR6843ISK.IWR6843ISK_people_counting_reader:main'
        ],
    },
) 