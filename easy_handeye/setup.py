from setuptools import setup, find_packages

package_name = 'easy_handeye'

setup(
    name=package_name,
    version='0.4.3',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Esposito',
    maintainer_email='esposito@imfusion.com',
    description='Simple, hardware-independent ROS2 library for hand-eye calibration',
    license='LGPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate = easy_handeye.calibrate:main',
            'publish = easy_handeye.publish:main',
            'robot = easy_handeye.robot:main',
            'handeye_calibration_commander = easy_handeye.handeye_calibration_commander:main',
        ],
    },
)

