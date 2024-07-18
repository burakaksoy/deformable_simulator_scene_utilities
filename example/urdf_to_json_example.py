import os
import sys

# This line inserts the package directory at the start of the system path
# Assuming your test scripts are being run from the `test` directory
package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, package_path)

from deformable_simulator_scene_utilities import urdf_to_json, urdf_str_to_json

file_name = "l_shape_corridor_width_0.7"

# IMPORT DIRECTLY FROM FILE
input_file_path = os.path.join("./", file_name + ".urdf")

output_file_path = os.path.join("./", file_name + ".json")

json_str = urdf_to_json(input_file_path=input_file_path, 
                        save_output=True, output_file_path=output_file_path, 
                        visualize=True)

print("Generated JSON from URDF file:", json_str)
print("----------------------------------------------------")

# print(urdf_str_to_json(urdf_str))
# print(urdf_str_to_json(urdf_str, visualize=True))







