from setuptools import find_packages, setup

package_name = 'bender_tf2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin-398',
    maintainer_email='jsimeone0105@gmail.com',
    description='Bender Bradley University TF2 localization package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bender_tag_localizer = bender_tf2.bender_tag_localizer:main'
        ],
    },
)
