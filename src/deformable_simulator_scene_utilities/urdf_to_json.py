import os
import io
import json

import numpy as np

import yourdfpy

"""
urdf_to_json.py: Transforms URDF files into JSON format for deformable object simulators.

This module processes URDF files, typically used in ROS environments, converting them into a JSON format that is suitable for use with deformable object simulators. 
The transformation includes complex parsing and computation to ensure that all relevant URDF elements are accurately represented in the JSON structure.

Features and Process:
- Parses URDF to extract link and joint data, converting each link into a JSON "RigidBody" object.
- Calculates transformations (translations and rotations) relative to the root link, ensuring correct spatial relationships.
- Handles multiple visual and collision geometries per link, including their origins.
- Converts primitive geometries (box, cylinder, sphere) specified in URDF to corresponding mesh files located in a predefined 'primitives' directory.
- Generates default values for necessary simulation parameters not typically included in URDF, such as dynamic properties and SDF resolutions.
- Outputs a well-formatted JSON file with comprehensive scene description, including custom metadata fields for enhanced simulation fidelity.

Usage:
The script requires the path to the URDF file and optionally outputs the resultant JSON to a specified file. 
It supports visualization of the parsed URDF model for verification purposes.

Dependencies:
Utilizes numpy for numerical operations and yourdfpy for URDF parsing. 
Ensure the required Python packages are installed, 
including yourdfpy[full] for full functionality.

Example:
Invoke the urdf_to_json function with appropriate parameters to convert an URDF file to JSON format, specifying output options as needed.

Design Considerations:
- The script identifies the root link by detecting which link is not a child in any joint definition.
- It correctly handles scenarios with multiple disconnected trees of links, ensuring that each tree's root transformations are computed accurately.
- The script ignores non-visual links, non-fixed joints, and other URDF elements like inertials, transmissions, 
and materials that are irrelevant to the JSON format needed for simulators.

Note: This script is part of a toolchain that facilitates the integration of robotic planning and simulation environments
by enabling seamless transitions between URDF and custom JSON formats used in specific simulators.
"""


# Get the directory of the current script
current_file_dir = os.path.dirname(os.path.abspath(__file__))
# Path to the primitives directory
primitives_dir = os.path.join(current_file_dir, 'meshes', 'primitives')

def R2rot(R):
    """
    Recover k and theta from a 3 x 3 rotation matrix
    
        sin(theta) = | R-R^T |/2
        cos(theta) = (tr(R)-1)/2
        k = invhat(R-R^T)/(2*sin(theta))
        theta = atan2(sin(theta),cos(theta)
        
    :type    R: numpy.array
    :param   R: 3 x 3 rotation matrix    
    :rtype:  (numpy.array, number)
    :return: ( 3 x 1 k unit vector, rotation about k in radians)   
    
    """
    def invhat(khat):
        return np.array([(-khat[1,2] + khat[2,1]),(khat[0,2] - khat[2,0]),(-khat[0,1]+khat[1,0])])/2
    
    R1 = R-R.transpose()
    
    sin_theta = np.linalg.norm(R1)/np.sqrt(8)
    
    cos_theta = (np.trace(R) - 1.0)/2.0
    theta = np.arctan2(sin_theta, cos_theta)
    
    #Avoid numerical singularity
    if sin_theta < 1e-6:
               
        if (cos_theta > 0):
            return [0,0,1], 0
        else:
            B = (1.0/2.0) *(R + np.eye(3))
            k = np.sqrt([B[0,0], B[1,1], B[2,2]])
            if np.abs(k[0]) > 1e-6:
                k[1] = k[1] * np.sign(B[0,1] / k[0])
                k[2] = k[2] * np.sign(B[0,2] / k[0])
            elif np.abs(k[1]) > 1e-6:
                k[2] = k[2] * np.sign(B[0,2] / k[1])
            return k, np.pi
    
    k = invhat(R1)/(2.0*sin_theta)    
    return list(np.squeeze(k)), theta
    
def _urdf_to_json(urdf_model, primitives_dir="./", visualize=False):
    # Validate the URDF model
    if urdf_model.validate():
        print("URDF model is valid")
    else:
        print("URDF model is not valid")
    # print("---------------------------------")
    
    if visualize:
        # Show the URDF model
        urdf_model.show()

    json_data = {}
    json_data["Name"] = urdf_model.base_link

    # We will create a list of rigid bodies from the visuals in the urdf
    # with increasing id numbers
    rigid_bodies = []
    id = 1

    for link_name, link_obj in urdf_model.link_map.items():
        # print("link_name: ", link_name)
        # print("link_obj: ", link_obj)
        # print("")
        
        transform_base_link_to_link = urdf_model.get_transform(frame_to=link_name,
                                                        frame_from=urdf_model.base_link, 
                                                        collision_geometry=False) # 4x4 list
        transform_base_link_to_link = np.array(transform_base_link_to_link) # 4x4 numpy array
        
        if link_obj.visuals:
            for visual in link_obj.visuals:
                rb_dict = {}
                rb_dict["id"] = id
                
                # Find the transform from the base_link to the visual origin
                if visual.origin is not None:
                    transform_link_to_visual = np.array(visual.origin)
                    transform_base_link_to_visual = np.dot(transform_base_link_to_link, transform_link_to_visual)
                else:
                    transform_base_link_to_visual = transform_base_link_to_link
                
                # print("link visual transform to base_link: ", transform_base_link_to_visual)
                
                rotation_axis, rotation_angle = R2rot(transform_base_link_to_visual[:3, :3])
                rb_dict["rotationAxis"] = rotation_axis
                rb_dict["rotationAngle"] = float(rotation_angle)
                rb_dict["translation"] = list(transform_base_link_to_visual[:3, 3])
                
                # Find the Geometry file
                if visual.geometry.box:
                    geometry_file = f"{primitives_dir}/box.obj"
                    
                    # prints if the path is not resolved
                    geometry_file = yourdfpy.filename_handler_magic(geometry_file, "/") # 
                    rb_dict["geometryFile"] = geometry_file
                    
                    rb_dict["scale"] = list(map(float, visual.geometry.box.size))
                    rb_dict["collisionObjectScale"] = rb_dict["scale"]
                    
                if visual.geometry.cylinder:
                    geometry_file = f"{primitives_dir}/cylinder.obj"
                    
                    # prints if the path is not resolved
                    geometry_file = yourdfpy.filename_handler_magic(geometry_file, "/") # 
                    rb_dict["geometryFile"] = geometry_file
                    
                    radius = float(visual.geometry.cylinder.radius)
                    length = float(visual.geometry.cylinder.length)
                    rb_dict["scale"] = [radius, radius, length]
                    rb_dict["collisionObjectScale"] = rb_dict["scale"]
                    
                if visual.geometry.sphere:
                    geometry_file = f"{primitives_dir}/sphere.obj"
                    
                    # prints if the path is not resolved
                    geometry_file = yourdfpy.filename_handler_magic(geometry_file, "/") # 
                    rb_dict["geometryFile"] = geometry_file
                    
                    radius = float(visual.geometry.sphere.radius)
                    rb_dict["scale"] = [radius, radius, radius]
                    rb_dict["collisionObjectScale"] = rb_dict["scale"]
                    
                if visual.geometry.mesh:
                    geometry_file = visual.geometry.mesh.filename
                    
                    # prints if the path is not resolved
                    geometry_file = yourdfpy.filename_handler_magic(geometry_file, "/") 
                    rb_dict["geometryFile"] = geometry_file
                    
                    
                    if visual.geometry.mesh.scale is None:
                        rb_dict["scale"] = [1, 1, 1]
                    else:
                        rb_dict["scale"] = list(visual.geometry.mesh.scale)
                        
                    rb_dict["collisionObjectScale"] = rb_dict["scale"]
                
                # Fill the rest of the metadata with the default values
                rb_dict["isDynamic"] = 0
                rb_dict["density"] = 1.0
                rb_dict["velocity"] = [0.0, 0.0, 0.0]
                rb_dict["angularVelocity"] = [0.0, 0.0, 0.0]
                rb_dict["restitution"] = 0.0
                rb_dict["frictionStatic"] = 0.5
                rb_dict["frictionDynamic"] = 0.5
                rb_dict["comment"] = "collisionObjectFileName can contain the path of an SDF file or if it is empty, the simulator will generate an SDF using the mesh in the geometryFile"
                rb_dict["collisionObjectFileName"] = ""
                rb_dict["resolutionSDF"] = [50, 50, 50]
                rb_dict["invertSDF"] = 0
                

                rigid_bodies.append(rb_dict)
                id += 1
        else:
            # print("---- No visuals in link ----")
            pass
                
        # print("---------------------------------")
        
    json_data["RigidBodies"] = rigid_bodies

    # print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    # print("Created json_data: ")

    json_str = json.dumps(json_data, indent=4)
    # print(json_str)
    
    return json_str

def urdf_to_json(input_file_path, 
                 save_output=False, output_file_path=None, 
                 visualize=False):
    
    # Check if the input file exists
    if not os.path.exists(input_file_path):
        print("Input file does not exist.")
        return None
    
    urdf_model = yourdfpy.URDF.load(input_file_path)
    
    json_str = _urdf_to_json(urdf_model, primitives_dir, visualize)
    
    if save_output:
        if not (output_file_path == "" or output_file_path is None):
            try:
                # Save the json_data to a file
                with open(output_file_path, "w") as file:
                    file.write(json_str)
                    print("Saved json data to file: ", output_file_path)
            except:
                print("Error saving json data to file: ", output_file_path)
        else:
            print("ERROR: No output file path provided")
            
    return json_str
    
def urdf_str_to_json(urdf_str, 
                     save_output=False, output_file_path=None, 
                     visualize=False):
    file_obj =  io.StringIO(urdf_str)
    urdf_model = yourdfpy.URDF.load(file_obj)
    
    json_str = _urdf_to_json(urdf_model, primitives_dir, visualize)
    
    if save_output:
        if not (output_file_path == "" or output_file_path is None):
            try:
                # Save the json_data to a file
                with open(output_file_path, "w") as file:
                    file.write(json_str)
                    print("Saved json data to file: ", output_file_path)
            except:
                print("Error saving json data to file: ", output_file_path)
        else:
            print("ERROR: No output file path provided")
            
    return json_str

