from setuptools import setup
package_name = 'xrce_link_demo'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Relay micro-ROS Int32 into ROS 2 topics and simple processing',
    entry_points={'console_scripts': [
        'relay = xrce_link_demo.relay_node:main',
        'processor = xrce_link_demo.processor_node:main',
        'jitter = xrce_link_demo.jitter_node:main',
    ]},
)
