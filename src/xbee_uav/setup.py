from setuptools import setup

package_name = 'xbee_uav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua Ashley',
    maintainer_email='jashley2017@gmail.com',
    description='ROS2 node for working with the XBEE radio as a node in a DigiMesh or Zigbee network. This is code that will go on the UAV.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radio = xbee_uav.radio:main',
        ],
    },
)
