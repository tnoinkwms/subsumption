from argparse import Namespace
from http.server import executable
from unicodedata import name
from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
import csv

#多分こいつにGAを書き込んでいけば，それなりに動くはず．．．
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
sensor_nodes =[
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
            executable = 'movingobj_detect'
        )
    ]
explore_nodes =[
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
            executable = 'avoid'
        )
    ]
wander_nodes = [
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'random_axis'
        ),
        Node(
            package =  'subsumption_architecture',
            namespace =  'subsumption_architecture',
            executable = 'periodic_axis'
        )
    ]
expression_nodes = [
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
        )
    ]
    
motor_nodes = [
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
        )
    ]

sensor_codes = [np.random.randint(0,2) for i in range(4)]
if sensor_codes[1] == 1:
    sensor_codes[2] = 1

explore = [np.random.randint(0,2) for i in range(3)]
wander = [np.random.randint(0,2) for i in range(2)]
expression = [np.random.randint(0,2) for i in range(3)]

def subsumption_check(explore,wander,expression):
    if explore == [0,0,0]:
        wander = [1,1]
    if wander == [0,0]:
        expression = [1,1,1]
    return explore,wander,expression

explore_codes, wander_codes , expression_codes = subsumption_check(explore,wander,expression)
launch_code = explore_codes + wander_codes + expression_codes 
launch_node = []

def generate_launch_description():
    for i in range(len(explore_codes)):
        if explore_codes[i] == 1:
            launch_node = np.append(launch_node,explore_nodes[i])
    for i in range(len(wander_codes)):
        if wander_codes[i] == 1:
            launch_node = np.append(launch_node,wander_nodes[i])
    for i in range(len(expression_codes)):
        if expression_codes[i] == 1:
            launch_node = np.append(launch_node,expression_nodes[i])
    launch_node = launch_node.tolist()
    launch_node = launch_node + motor_nodes
    with open('/home/tnoin/codes/subsumption/src/subsumption_architecture/launch/data_1.csv','a') as f:
        writer = csv.writer(f)
        writer.writerow(launch_code)

    return LaunchDescription(launch_node)
