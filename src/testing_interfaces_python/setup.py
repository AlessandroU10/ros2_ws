from setuptools import find_packages, setup

package_name = 'testing_interfaces_python'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'sphere_publisher = testing_interfaces_python.sphere_publisher:main',
                'sphere_listener = testing_interfaces_python.sphere_listener:main',
                'add_service_node = testing_interfaces_python.add_service_node:main',
                'add_client_node = testing_interfaces_python.add_client_node:main',

        ],
    },
)
