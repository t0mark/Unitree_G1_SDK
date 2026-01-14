from glob import glob

from setuptools import setup

package_name = 'synchronize'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/models/g1', glob('models/g1/*.xml')),
        ('share/' + package_name + '/models/g1/meshes', glob('models/g1/meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@unitree.com',
    description='Real robot to Mujoco simulation synchronization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'synchronize_node = synchronize.synchronize_node:main',
        ],
    },
)
