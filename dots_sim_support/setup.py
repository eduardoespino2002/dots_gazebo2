from setuptools import setup
from glob import glob

package_name = 'dots_sim_support'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simonj',
    maintainer_email='simon2.jones@bristol.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_dots_hardware = dots_sim_support.fake_dots_hardware:main',
            'urdf_prefix = dots_sim_support.urdf_prefix:main',
        ],
    },
)
