# URDF and SDF Formats

## Introduction

URDF (Unified Robot Description Format) and SDF (Simulation Description Format) are XML-based formats used to describe robots and simulation environments in robotics. Understanding these formats is crucial for creating and working with robot models in simulation environments like Gazebo.

## URDF (Unified Robot Description Format)

URDF is primarily used in ROS to describe robot models. It defines the physical and visual properties of a robot, including links, joints, and their relationships.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.3 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### URDF Elements

#### Links
Links represent rigid bodies in the robot:

```xml
<link name="link_name">
  <!-- Visual properties for rendering -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Supported geometries: box, cylinder, sphere, mesh -->
      <box size="1 1 1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>

  <!-- Inertial properties for dynamics -->
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

#### Joints
Joints define how links connect and move relative to each other:

```xml
<!-- Fixed joint (no movement) -->
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>

<!-- Revolute joint (rotation around one axis) -->
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>

<!-- Continuous joint (unlimited rotation) -->
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>

<!-- Prismatic joint (linear movement) -->
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="10.0" velocity="1.0"/>
</joint>
```

### Materials and Colors

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="black">
  <color rgba="0 0 0 1"/>
</material>

<material name="white">
  <color rgba="1 1 1 1"/>
</material>
```

### Transmission Elements

For controlling joints with actuators:

```xml
<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## SDF (Simulation Description Format)

SDF is used by Gazebo for simulation-specific descriptions. It can include URDF models but also provides additional simulation-specific features.

### Basic SDF Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="my_robot">
    <pose>0 0 0.5 0 0 0</pose>

    <!-- Links -->
    <link name="chassis">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.2</iyy>
          <iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box>
            <size>1.0 0.5 0.3</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>1.0 0.5 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>

    <!-- Joints -->
    <joint name="chassis_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>10.0</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

### SDF vs URDF Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| Primary Use | ROS robot description | Gazebo simulation |
| Kinematics | Yes | Yes |
| Dynamics | Limited | Comprehensive |
| Sensors | Through plugins | Built-in support |
| Controllers | Through ROS | Built-in support |
| World Description | No | Yes |
| Physics Properties | Limited | Extensive |

## Converting Between URDF and SDF

### Using Robot State Publisher with Gazebo

Often, URDF models are used in Gazebo by converting them at runtime:

```xml
<!-- In a launch file -->
<node name="robot_state_publisher" pkg="robot_state_publisher" exec="robot_state_publisher">
  <param name="robot_description" value="$(var robot_description_file)"/>
</node>

<!-- Spawn the robot in Gazebo -->
<node name="spawn_entity" pkg="gazebo_ros" exec="spawn_entity.py"
      args="-topic robot_description -entity my_robot"/>
```

### Manual Conversion Considerations

When converting from URDF to SDF for simulation:

1. Add simulation-specific properties (damping, friction)
2. Include Gazebo-specific plugins
3. Define physics parameters more precisely
4. Add sensor configurations if needed

## Gazebo-Specific Extensions in URDF

You can embed Gazebo-specific elements within URDF:

```xml
<robot name="my_robot">
  <!-- Standard URDF elements -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>false</self_collide>
    <gravity>true</gravity>
    <max_contacts>10</max_contacts>
  </gazebo>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.15</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</robot>
```

## Practical Example: Simple Mobile Robot

Here's a complete URDF for a simple differential drive robot:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.14583" ixy="0.0" ixz="0.0" iyy="0.14583" iyz="0.0" izz="0.225"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00125" ixy="0.0" ixz="0.0" iyy="0.00125" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00125" ixy="0.0" ixz="0.0" iyy="0.00125" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.25 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="base_to_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.25 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>base_to_left_wheel</left_joint>
      <right_joint>base_to_right_wheel</right_joint>
      <wheel_separation>0.5</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</robot>
```

## Xacro for Complex Models

Xacro (XML Macros) allows for more complex and reusable robot descriptions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">
  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="wheel_separation" value="0.5" />

  <!-- Macro for wheel -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 0 1"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.00125" ixy="0.0" ixz="0.0" iyy="0.00125" iyz="0.0" izz="0.0025"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Robot body -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.14583" ixy="0.0" ixz="0.0" iyy="0.14583" iyz="0.0" izz="0.225"/>
    </inertial>
  </link>

  <!-- Use the wheel macro -->
  <xacro:wheel prefix="left" parent="base_link" xyz="0 ${wheel_separation/2} -0.05" rpy="${M_PI/2} 0 0"/>
  <xacro:wheel prefix="right" parent="base_link" xyz="0 -${wheel_separation/2} -0.05" rpy="${M_PI/2} 0 0"/>
</robot>
```

## Validation and Testing

### Validating URDF Files

```bash
# Check URDF syntax
check_urdf my_robot.urdf

# Parse and display robot information
urdf_to_graphiz my_robot.urdf
```

### Loading URDF in RViz2

```xml
<!-- Launch file to load and visualize URDF -->
<launch>
  <param name="robot_description" value="$(find-pkg-share my_robot_pkg)/urdf/my_robot.urdf"/>
  <node pkg="robot_state_publisher" exec="robot_state_publisher"/>
  <node pkg="rviz2" exec="rviz2"/>
</launch>
```

## Best Practices

1. **Use consistent units**: Always use meters for length, kilograms for mass
2. **Realistic inertial properties**: Calculate or estimate properly for stable simulation
3. **Simple collision geometry**: Use simpler shapes for collision than visual models
4. **Organize with Xacro**: Use macros for repeated elements
5. **Validate regularly**: Check your URDF with validation tools
6. **Document assumptions**: Comment on design decisions and limitations

## Summary

URDF and SDF are fundamental formats for describing robots and simulation environments. URDF is primarily used in ROS for robot description, while SDF is used by Gazebo for simulation-specific features. Understanding both formats and how to use them effectively is essential for creating accurate and functional robot models in simulation.

In the next section, we'll explore physics and sensor simulation in detail.