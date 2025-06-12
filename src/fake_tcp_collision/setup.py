from setuptools import find_packages, setup

package_name = 'fake_tcp_collision'

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
    maintainer='liyq',
    maintainer_email='pei_liu0913@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'fake_tcp_node = fake_tcp_collision.fake_tcp_node:main',
        ],
    },
)
