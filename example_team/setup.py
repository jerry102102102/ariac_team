from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'example_team'

# Collect every file inside config/, keeping their internal structure
config_files = [
    (
        os.path.join('share', package_name, os.path.dirname(f)),
        [f],
    )
    for f in glob('config/**', recursive=True)
    if os.path.isfile(f)
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
    ] + config_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Justin Albrecht',
    maintainer_email='justin.albrecht@nist.gov',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robots = example_team.examples.move_robots:main',
            'dummy_inspection = example_team.examples.dummy_inspection:main',
            'pick_from_tester = example_team.examples.pick_from_tester:main'
        ],
    },
)
