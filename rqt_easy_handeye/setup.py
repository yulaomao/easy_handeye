from setuptools import setup, find_packages

package_name = 'rqt_easy_handeye'

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
    description='rqt GUI for the easy_handeye package',
    license='LGPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_easy_handeye = rqt_easy_handeye.rqt_easy_handeye:main',
            'rqt_calibrationmovements = rqt_easy_handeye.rqt_calibrationmovements:main',
            'rqt_calibration_evaluator = rqt_easy_handeye.rqt_calibration_evaluator:main',
        ],
    },
)
