from setuptools import setup

package_name = 'open3d_conversions'

setup(
        name=package_name,
        version='0.0.1',
        packages=[package_name],
        data_files=[
                ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Marco Esposito',
        maintainer_email='esposito@imfusion.com',
        description='Conversions between ROS types and Open3D point clouds',
        license='LGPLv3',
        tests_require=['pytest'],
        entry_points={
                'console_scripts': [
                ],
        },
)

