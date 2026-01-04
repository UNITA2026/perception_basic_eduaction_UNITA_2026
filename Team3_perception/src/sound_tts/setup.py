from setuptools import setup
from glob import glob
import os

package_name = 'sound_tts'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ROS2 패키지 인식용
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # 🔊 mp3 파일들을 share로 설치 (중요)
        (os.path.join('share', package_name, 'sound'),
         glob('sound/*.mp3')),
         
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jkm',
    maintainer_email='jkm@todo.todo',
    description='Sound TTS node using pygame mixer',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run sound_tts sound_node
            'sound_node = sound_tts.sound_node:main',
        ],
    },
)
