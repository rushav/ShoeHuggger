from setuptools import setup

package_name = 'alienbot_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rushav',
    maintainer_email='rushav@uw.edu',
    description='AlienBot vision and control nodes for person following with YOLOv8',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'person_follower = alienbot_vision.person_follower:main',
            'person_follower_calm = alienbot_vision.person_follower_calm:main',
            'teleop_hold = alienbot_vision.teleop_hold:main',
            'alienbot_ui = alienbot_vision.alienbot_ui:main',
            'person_detector = alienbot_vision.person_detector:main',
        ],
    },
)
