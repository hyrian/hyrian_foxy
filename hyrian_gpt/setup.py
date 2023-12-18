from setuptools import setup

package_name = 'hyrian_gpt'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='jungwoohyeon@gmail.com',
    description='A ROS 2 package for GPT interaction',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpt_pub_test = gpt_test.gpt_pub_test:main',
            'keyword_sub = gpt_test.keyword_sub:main',
        ],
    },
)
