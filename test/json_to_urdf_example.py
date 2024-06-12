import os
import sys

# This line inserts the package directory at the start of the system path
# Assuming your test scripts are being run from the `test` directory
package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, package_path)

from deformable_simulator_scene_utilities import json_to_urdf, json_str_to_urdf


## IMPORT FROM JSON STRING
json_str = """
{
    "Name": "corridor_base_link",
    "RigidBodies": [
        {
            "id": 1,
            "rotationAxis": [
                0,
                0,
                1
            ],
            "rotationAngle": 0.0,
            "translation": [
                0.0,
                0.0,
                -0.05
            ],
            "geometryFile": "/home/burak/catkin_ws_deformable/src/deformable_description/deformable_simulator_scene_utilities/src/deformable_simulator_scene_utilities/meshes/primitives/box.obj",
            "scale": [
                6.0,
                6.0,
                0.1
            ],
            "collisionObjectScale": [
                6.0,
                6.0,
                0.1
            ],
            "isDynamic": 0,
            "density": 1.0,
            "velocity": [
                0.0,
                0.0,
                0.0
            ],
            "angularVelocity": [
                0.0,
                0.0,
                0.0
            ],
            "restitution": 0.0,
            "frictionStatic": 0.5,
            "frictionDynamic": 0.5,
            "comment": "collisionObjectFileName can contain the path of an SDF file or if it is empty, the simulator will generate an SDF using the mesh in the geometryFile",
            "collisionObjectFileName": "",
            "resolutionSDF": [
                50,
                50,
                50
            ],
            "invertSDF": 0
        },
        {
            "id": 2,
            "rotationAxis": [
                0,
                0,
                1
            ],
            "rotationAngle": 0.0,
            "translation": [
                0.0,
                2.95,
                1.2
            ],
            "geometryFile": "/home/burak/catkin_ws_deformable/src/deformable_description/deformable_simulator_scene_utilities/src/deformable_simulator_scene_utilities/meshes/primitives/box.obj",
            "scale": [
                6.0,
                0.1,
                2.4
            ],
            "collisionObjectScale": [
                6.0,
                0.1,
                2.4
            ],
            "isDynamic": 0,
            "density": 1.0,
            "velocity": [
                0.0,
                0.0,
                0.0
            ],
            "angularVelocity": [
                0.0,
                0.0,
                0.0
            ],
            "restitution": 0.0,
            "frictionStatic": 0.5,
            "frictionDynamic": 0.5,
            "comment": "collisionObjectFileName can contain the path of an SDF file or if it is empty, the simulator will generate an SDF using the mesh in the geometryFile",
            "collisionObjectFileName": "",
            "resolutionSDF": [
                50,
                50,
                50
            ],
            "invertSDF": 0
        },
        {
            "id": 3,
            "rotationAxis": [
                -2.653594666942124e-06,
                -4.8735987367015545e-12,
                -0.9999999999964793
            ],
            "rotationAngle": 1.5707926535968348,
            "translation": [
                2.95,
                1.5,
                1.2
            ],
            "geometryFile": "/home/burak/catkin_ws_deformable/src/deformable_description/deformable_simulator_scene_utilities/src/deformable_simulator_scene_utilities/meshes/primitives/box.obj",
            "scale": [
                3.0,
                0.1,
                2.4
            ],
            "collisionObjectScale": [
                3.0,
                0.1,
                2.4
            ],
            "isDynamic": 0,
            "density": 1.0,
            "velocity": [
                0.0,
                0.0,
                0.0
            ],
            "angularVelocity": [
                0.0,
                0.0,
                0.0
            ],
            "restitution": 0.0,
            "frictionStatic": 0.5,
            "frictionDynamic": 0.5,
            "comment": "collisionObjectFileName can contain the path of an SDF file or if it is empty, the simulator will generate an SDF using the mesh in the geometryFile",
            "collisionObjectFileName": "",
            "resolutionSDF": [
                50,
                50,
                50
            ],
            "invertSDF": 0
        }
    ]
}
"""


output_dir = "./"
output_file_path = os.path.join(output_dir, "example_output_from_json_string.urdf")

urdf_str = json_str_to_urdf(json_str=json_str, 
                            save_output=True, output_file_path=output_file_path, 
                            visualize=True)

print("Generated URDF from JSON string:", urdf_str)
print("----------------------------------------------------")

# IMPORT DIRECTLY FROM FILE
input_file_path = os.path.join("./", "example.json")
output_file_path = os.path.join(output_dir, "example_output_from_json_file.urdf")

urdf_str = json_to_urdf(input_file_path=input_file_path, 
                        save_output=True, output_file_path=output_file_path, 
                        visualize=True)

print("Generated URDF from JSON file:", urdf_str)
print("----------------------------------------------------")

# print(json_str_to_urdf(json_str))
# print(json_str_to_urdf(json_str, visualize=True))

