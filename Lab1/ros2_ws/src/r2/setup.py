from setuptools import setup

package_name = 'r2'

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
    maintainer='dcs',
    maintainer_email='sssp3095076964@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prt = r2.printer:main',
            'pub = r2.publisher:main',
            'sub = r2.subscriber:main'
        ],
    },
)
