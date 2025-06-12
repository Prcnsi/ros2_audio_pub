from setuptools import setup

package_name = 'ros2_audio_pub'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/AudioWithDOA.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Audio + DOA publisher',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mic_audio_with_doa = ros2_audio_pub.mic_audio_with_doa:main',
        ],
    },
)
