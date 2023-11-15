from setuptools import find_packages
from setuptools import setup

package_name = 'tm_image'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leowu',
    maintainer_email='leo.wu@tm-robot.com',
    description='tm_image',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_talker = tm_image.image_pub:main',
            'status_talker = tm_image.get_status:main'
        ],
    },
)
