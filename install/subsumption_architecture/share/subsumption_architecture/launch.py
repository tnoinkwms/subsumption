from argparse import Namespace
from http.server import executable
from unicodedata import name
from launch import LaunchDescription
from launch_ros.actions import Node

#多分こいつにGAを書き込んでいけば，それなりに動くはず．．．
def generate_launch_description():
    node_list = [
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'face_detect'
        ),
         Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'hand_pos'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'hand_detect'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'PID_eye'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'PID_neck'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'movingobj_detect'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'avoid'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'random_axis'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'periodic_axis'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'face_expression'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'alife_engine'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'imitation'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'camera_pub'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'camera_sub'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'sim'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'main'
        ),       
    ]
    launch_node = [node_list[7],node_list[14]]
    return LaunchDescription(node_list)
