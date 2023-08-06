from setuptools import setup

package_name = 'pa1'

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
    maintainer='rvilab',
    maintainer_email='rvilab@todo.todo',
    description='PA1',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "publish_time = pa1.PublishTimeNode:main",
            "subscribe_time = pa1.SubscribeTimeNode:main",
            "fitler_img = pa1.Filtering:main"
        ],
    },
)
