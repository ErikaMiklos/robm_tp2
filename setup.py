from setuptools import setup

package_name = 'robm_tp2'

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
    maintainer='Vincent Drevelle',
    maintainer_email='vincent.drevelle@univ-rennes1.fr',
    description='Outils et squelettes de code pour le TP2 de ROBM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_controller = robm_tp2.base_controller:main',
            'odometry = robm_tp2.odometry:main',
        ],
    },
)
