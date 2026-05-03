from launch import LaunchDescription
from launch_ros.actions import Node
import math 

def generate_launch_description(): 
    ld = LaunchDescription() 

    node = Node( 

        package="tf2_ros", 
        
        executable="static_transform_publisher", 

        output="screen", 

        arguments=[ "0.0175", "0.10442", "0.05673",  # Translation (x, y=0.09942, z = 0.05173) 
            "-0.5","-0.5","-0.5","0.5",  # Quaternion (x, y, z, w) rotation along y -90 and z -90
            "wrist_3_link", # Parent frame
            "camera_link"  # child frame  
            ] 

    ) 
    
    ld.add_action(node) 
    return ld 


