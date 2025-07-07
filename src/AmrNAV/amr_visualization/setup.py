from setuptools import setup

package_name = 'amr_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='你的名字',
    maintainer_email='你的信箱',
    description='AMR visualization tools including marker publisher.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_publisher = amr_visualization.marker_publisher:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/amr_visualization']),
        ('share/' + package_name, ['package.xml']),
    ],
)

