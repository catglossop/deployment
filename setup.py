from setuptools import setup
from glob import glob
import os

package_name = 'deployment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'hierarchical_learning'), glob('hierarchical_learning/*.py')),
        (os.path.join('lib', package_name, 'models'), glob('../hierarchical_learning/models/*/*.py')),
        (os.path.join('lib', package_name, 'deployment'), glob('deployment/**/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='create',
    maintainer_email='create@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_utils_node = deployment.teleop_utils:main',
            'create_topomap_node = deployment.hierarchical_learning.create_topomap:main', 
            'low_level_policy = deployment.hierarchical_learning.low_level_policy:main', 
            'pd_controller_node = deployment.hierarchical_learning.pd_controller:main'
        ],
    },
)
