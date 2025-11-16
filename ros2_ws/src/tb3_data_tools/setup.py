from setuptools import setup

package_name = 'tb3_data_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/record_tb3_dataset.launch.py']),
        ('share/' + package_name + '/config', ['config/recorder_topics.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SLAM Dev',
    maintainer_email='dev@example.com',
    description='Recorder nodes and launch files for TurtleBot3 dataset capture.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_topic_recorder = tb3_data_tools.data_recorder_node:main',
            'sync_bag_runner = tb3_data_tools.sync_bag_runner:main'
        ],
    },
)
