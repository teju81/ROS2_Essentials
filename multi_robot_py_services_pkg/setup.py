from setuptools import find_packages, setup

package_name = 'multi_robot_py_services_pkg'

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
    maintainer='raviteja',
    maintainer_email='teju81@gmail.com',
    description='Package for Python based Service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = multi_robot_py_services_pkg.multi_robot_py_services_server:main',
            'client = multi_robot_py_services_pkg.multi_robot_py_services_client:main',
        ],
    },
)
