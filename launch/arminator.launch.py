from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for AL5D robot arm'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate for publishing robot state (Hz)'
    )
    
    # AL5D controller node
    arminator_node = Node(
        package='arminator',
        executable='arminator_node',
        name='arminator_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'publish_rate': LaunchConfiguration('publish_rate')
        }]
    )
    
    # Robot state publisher for visualization (optional)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': get_robot_description()}]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        publish_rate_arg,
        arminator_node,
        robot_state_publisher
    ])

def get_robot_description():
    """
    Basic URDF description for AL5D robot arm
    This is a simplified model - you may want to create a more detailed URDF file
    """
    return """<?xml version="1.0"?>
<robot name="al5d">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="base_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  
  <link name="shoulder_link">
    <visual>
      <geometry>
        <box size="0.1 0.05 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="shoulder_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.075" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  
  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.146" radius="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0.146" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  
  <link name="forearm_link">
    <visual>
      <geometry>
        <cylinder length="0.187" radius="0.015"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="wrist_joint" type="revolute">
    <parent link="forearm_link"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.187" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  
  <link name="wrist_link">
    <visual>
      <geometry>
        <box size="0.05 0.03 0.05"/>
      </geometry>
      <material name="yellow">
        <color rgba="0.8 0.8 0.2 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="wrist_rotate_joint" type="revolute">
    <parent link="wrist_link"/>
    <child link="gripper_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  
  <link name="gripper_link">
    <visual>
      <geometry>
        <box size="0.08 0.02 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="gripper_joint" type="revolute">
    <parent link="gripper_link"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="3.14" effort="10" velocity="1"/>
  </joint>
  
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>
</robot>"""