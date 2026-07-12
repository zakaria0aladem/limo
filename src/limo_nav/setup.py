from setuptools import setup

package_name = 'limo_nav'

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
    maintainer='Zakaria',
    maintainer_email='zakaria@example.com',
    description='Reactive potential-field wandering node for the LIMO Pro.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wandering = limo_nav.wandering:main'
        ],
    },
)
