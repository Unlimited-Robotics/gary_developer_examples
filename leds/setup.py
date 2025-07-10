from setuptools import setup

package_name = 'leds'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=['led_node'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Maintainer Name',
    maintainer_email='maintainer@example.com',
    description="LED node example package for Gary's LED system.",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_node = led_node:main',
        ],
    },
)
