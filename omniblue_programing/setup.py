from setuptools import setup

package_name = 'omniblue_programing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zmtech',
    maintainer_email='zmtech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'omniblue_move = omniblue_programing.omniblue_move:main',
        'omniblue_marker = omniblue_programing.omniblue_marker:main',
        'omniblue_marked_move = omniblue_programing.omniblue_marked_move:main',
        'omniblue_marked_optimal_navigation = omniblue_programing.omniblue_marked_optimal_move:main',
 
        ],
    },
)
