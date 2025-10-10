from setuptools import setup, find_packages

package_name = 'ariac_team'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='.'),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ariac_team']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mvp_smoke.launch.py']),
        ('share/' + package_name + '/config', ['config/ariac_team_config.yaml']),
        ('share/' + package_name + '/config/trials', [
            'config/trials/mvp_smoke.yaml',
            'config/trials/conveyor_to_tester.yaml',
            'config/trials/tester_to_agv.yaml',
            'config/trials/shipping_submit.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ariac_team',
    maintainer_email='team@example.com',
    description='Team specific MVP pipeline for ARIAC 2025.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mvp_coordinator = ariac_team.nodes.mvp_coordinator:main',
        ],
    },
)
