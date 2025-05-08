from setuptools import find_packages, setup

package_name = 'turtlesim_catch_them_all'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olawalepeace',
    maintainer_email='olawalepeace0@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_manager = turtlesim_catch_them_all.turtle_manager:main',
            'master_turtle_controller = turtlesim_catch_them_all.master_turtle_controller:main'
        ],
    },
)
