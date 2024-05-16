from setuptools import find_packages
from setuptools import setup
from glob import glob

package_name = 'trimble_gnss_driver_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/', glob('config/*', recursive=True)),
        ('share/' + package_name + '/launch/', glob('launch/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Geesara, Ibrahim',
    maintainer_email='ggeesara@gmail.com, ibrahim.hroub7@gmail.com',
    description='The ros2 trimble_gnss_driver_ros2 package',
    license='Apache-2.0 license',
    tests_require=['pytest', 'launch-pytest'],
    entry_points={
        'console_scripts': [
            'gsof_driver = trimble_gnss_driver_ros2.gsof_driver:main',
        ],
    },

)
