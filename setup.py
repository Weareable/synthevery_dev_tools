import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'synthevery_dev_tools'

# build a list of the data files
data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))

def package_files(directory, data_files):
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            data_files.append(("share/" + package_name + "/" + path, glob(path + "/**/*.*", recursive=True)))
    return data_files

data_files = package_files('launch/', data_files)
data_files = package_files('meshes/', data_files)
data_files = package_files('config/', data_files)
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wachinaba',
    maintainer_email='wachinaba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_publisher = synthevery_dev_tools.serial_publisher:main',
            'synthevery_serial_bridge = synthevery_dev_tools.synthevery_serial_bridge:main',
            'adaptive_kp_mahony_orientation_filter = synthevery_dev_tools.adaptive_kp_mahony_orientation_filter:main',
            'global_accel_publisher = synthevery_dev_tools.global_accel_publisher:main',
            'mesh_publisher = synthevery_dev_tools.mesh_publisher:main',
            'state_machine = synthevery_dev_tools.state_machine:main',
        ],
    },
)