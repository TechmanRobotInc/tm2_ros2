from setuptools import find_packages, setup

package_name = 'tm_image'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'flask',
        'waitress',
        'numpy',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='leo wu, Yh Wang',
    maintainer_email='leo.wu@tm-robot.com, yh.wang@tm-robot.com',
    description='tm_image',
    license='BSD-3-Clause',
    # tests_require=['pytest'],
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'image_talker = tm_image.image_pub:main',
            'status_talker = tm_image.get_status:main'
        ],
    },
)
