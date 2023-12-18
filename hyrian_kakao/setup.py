from setuptools import setup

package_name = 'hyrian_kakao'

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
    maintainer='wonwoo',
    maintainer_email='neal0907@hanyang.com',
    description='Kakao Payment service node in Hyrian Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "kakaopay_node1 = hyrian_kakao.kakaopay_node1:main"
        ],
    },
)
