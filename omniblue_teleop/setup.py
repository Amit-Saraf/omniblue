from setuptools import setup

package_name = 'omniblue_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zmtech',
    maintainer_email='qaz9517532846@gmail.com',
    description='Teleoperation node using keyboard for zm_robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'omniblue_teleop_key = omniblue_teleop.omniblue_teleop_key:main'
        ],
    },
)
