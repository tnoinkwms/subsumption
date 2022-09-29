import os
from setuptools import setup
import os
from glob import glob

package_name = 'subsumption_architecture'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tnoin',
    maintainer_email='tnoin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detect = subsumption_architecture.face:main',
            'hand_pos = subsumption_architecture.send_hand_pos:main',
            'hand_detect = subsumption_architecture.handgaze:main',
            'movingobj_detect = subsumption_architecture.objdetect:main',
            'PID_eye = subsumption_architecture.PID_eye:main',
            'PID_neck = subsumption_architecture.PID_neck:main',
            'avoid = subsumption_architecture.avoid:main',
            'random_axis = subsumption_architecture.randomaxis:main',
            'periodic_axis = subsumption_architecture.periodicaxis:main',
            'face_expression = subsumption_architecture.faceexpression:main',
            'alife_engine = subsumption_architecture.AlifeEngine:main',
            'imitation = subsumption_architecture.imitation:main',
            'camera_pub = subsumption_architecture.pub:main',
            'camera_sub = subsumption_architecture.sub:main',
            'main = subsumption_architecture.server:main',
            'sim = subsumption_architecture.server:main_sim'
        ],
    },
)
