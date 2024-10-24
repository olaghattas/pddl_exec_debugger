from setuptools import find_packages, setup

package_name = 'pddl_debugger_interface'

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
    maintainer='olagh48652',
    maintainer_email='olaghattas@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_interface = pddl_debugger_interface.gui_interface:main',
            'gui_interface_kb = pddl_debugger_interface.gui_interface_kb_add:main',
            'service_test = pddl_debugger_interface.service_test:main',
        ],
    },
)
