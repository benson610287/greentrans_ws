from setuptools import find_packages, setup

package_name = 'path_pkg'

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
    maintainer='tku-5080',
    maintainer_email='tku-5080@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "main=path_pkg.main:main",
            "path_checker=path_pkg.path_checker:main",
            "test_version=path_pkg.test_version:main"
        ],
    },
)
