from setuptools import find_packages, setup

package_name = 'multi_robot_py_topics_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raviteja U.',
    maintainer_email='teju81@gmail.com',
    description='Package for Multi Robot Py Publishers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = multi_robot_py_topics_pkg.multi_robot_publisher:main',
                'listener = multi_robot_py_topics_pkg.multi_robot_subscriber:main',
        ],
    },
)
