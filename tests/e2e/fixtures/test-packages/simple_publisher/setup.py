from setuptools import setup

package_name = 'simple_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Test Maintainer',
    maintainer_email='test@robocore.dev',
    description='Simple test publisher for robocore-cli e2e tests',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = simple_publisher.publisher:main',
        ],
    },
)