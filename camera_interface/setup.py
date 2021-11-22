from setuptools import setup

package_name = 'camera_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrew',
    maintainer_email='andrewoshei@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_subscriber = camera_interface.camera_sub_node:main',
            'search_for_object = camera_interface.search_for_object:main',
            'got_to_objrct = camera_interface.go_to_object:main'
        ],
    },
)
