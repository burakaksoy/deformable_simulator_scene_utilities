import os
import sys

# This line inserts the package directory at the start of the system path
# Assuming your test scripts are being run from the `test` directory
package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, package_path)

from deformable_simulator_scene_utilities import urdf_to_json, urdf_str_to_json


## IMPORT FROM URDF STRING
urdf_str ="""<?xml version="1.0"?>
<robot name="a">
    <link name="corridor_base_link"/>

    <link name="link_ground_plane">
        <visual>
            <material name="">
                <color rgba="0.8 0.8 0.5 1"/>
            </material>
            <geometry>
                <box size="6 6 0.1"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="6 6 0.1"/>
            </geometry>
        </collision>
    </link>

    <joint name="joint_ground_plane" type="fixed">
        <origin xyz="0 0 -0.05"/>
        <parent link="corridor_base_link"/>
        <child link="link_ground_plane"/>
    </joint>

    <link name="outer_l_shaped_wall_origin_link"/>

    <joint name="outer_l_shaped_wall_origin_joint" type="fixed">
        <origin xyz="3 3 0"/>
        <parent link="corridor_base_link"/>
        <child link="outer_l_shaped_wall_origin_link"/>
    </joint>
    
    
    <link name="outer_along_x_wall_link">
        <visual>
            <origin xyz="0 0 0"/>
            <material name="">
                <color rgba="0.5 0.5 0.5 0.5"/>
            </material>
            <geometry>
                <box size="6 0.1 2.4"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="6 0.1 2.4"/>
            </geometry>
        </collision>
    </link>

    <joint name="outer_along_x_wall_link_joint" type="fixed">
        <origin xyz="-3 -0.05 1.2"/>
        <parent link="outer_l_shaped_wall_origin_link"/>
        <child link="outer_along_x_wall_link"/>
    </joint>
    
    <link name="outer_along_y_wall_link">
        <visual>
            <material name="">
                <color rgba="0.5 0.5 0.5 0.5"/>
            </material>
            <geometry>
                <box size="3 0.1 2.4"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="3 0.1 2.4"/>
            </geometry>
        </collision>
    </link>
    
    
    <joint name="outer_along_y_wall_link_joint" type="fixed">
        <origin xyz="-0.05 -1.5 1.2" rpy="3.14159 -3.14159 1.5708"/>
        <parent link="outer_l_shaped_wall_origin_link"/>
        <child link="outer_along_y_wall_link"/>
    </joint>
    
</robot>
"""


output_dir = "./"
output_file_path = os.path.join(output_dir, "example_output_from_urdf_string.json")

json_str = urdf_str_to_json(urdf_str=urdf_str, 
                            save_output=True, output_file_path=output_file_path, 
                            visualize=True)

print("Generated JSON from URDF string:", json_str)
print("----------------------------------------------------")

# IMPORT DIRECTLY FROM FILE
input_file_path = os.path.join("./", "example.urdf")
output_file_path = os.path.join(output_dir, "example_output_from_urdf_file.json")

json_str = urdf_to_json(input_file_path=input_file_path, 
                        save_output=True, output_file_path=output_file_path, 
                        visualize=True)

print("Generated JSON from URDF file:", json_str)
print("----------------------------------------------------")


# # IMPORT DIRECTLY FROM FILE
# input_file_path = "/home/burak/catkin_ws_deformable/src/deformable_description/urdf/scenes_mingrui_yu/scene_mingruiyu_7_narrow.urdf"

# output_file_path = "/home/burak/catkin_ws_deformable/src/dlo_simulator_stiff_rods/config/scenes/scene_mingruiyu_7_narrow.json"

# json_str = urdf_to_json(input_file_path=input_file_path, 
#                         save_output=True, output_file_path=output_file_path, 
#                         visualize=True)

# print("Generated JSON from URDF file:", json_str)
# print("----------------------------------------------------")