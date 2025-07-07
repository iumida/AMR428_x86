from setuptools import setup

package_name = 'amr_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='你的名字',
    maintainer_email='你的信箱',
    description='AMR robot description with marker publisher',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_publisher = amr_description.marker_publisher:main'
        ],
    },
)
