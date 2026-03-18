from setuptools import setup

package_name = 'alienbot_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'teleop_hold = alienbot_vision.teleop_hold:main',
            'person_follower = alienbot_vision.person_follower:main',
            'alienbot_ui = alienbot_vision.alienbot_ui:main',
            'person_follower_calm = alienbot_vision.person_follower_calm:main',
        ],
    },
)