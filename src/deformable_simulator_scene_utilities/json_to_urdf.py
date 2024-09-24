import os
import io
import json

import numpy as np

from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

import yourdfpy

"""
json_to_urdf.py: Converts JSON scene descriptions to URDF files for ROS environments.

Author: Burak Aksoy

This module provides functionality to convert JSON formatted scene descriptions,
typically used in deformable object simulators, into URDF files compatible with
ROS and Tesseract Planners. It reads a JSON file, extracts scene and object
parameters, and outputs a well-formatted URDF file.

Key Features:
- Root link is named after the 'Name' field in the JSON, serving as an empty root
  for the scene hierarchy.
- Each entry in 'RigidBodies' is converted into a URDF link with a fixed joint
  to the root. Link names are derived from the geometry file names and include
  unique identifiers.
- Transforms (translation and rotation) are applied as specified in the JSON.
- Additional metadata from JSON (like density, friction coefficients) are
  embedded as custom XML elements within each link for comprehensive simulation
  detail.
- Mesh file paths are prefixed with 'file://' to conform to URI standards required
  by ROS and Tesseract environments.

Usage:
To generate a URDF from a JSON scene description, ensure that the JSON file is
formatted correctly with all necessary fields. The output URDF is readable,
with appropriate indentations and line breaks, enhancing usability and maintainability.

Example:
Run the function json_to_urdf with the path to your JSON file to generate and
optionally visualize the URDF structure.

Dependencies:
Requires numpy for matrix operations and xml.etree for XML handling. Ensure
yourdfpy[full] is installed for full functionality.

"""

def _save_urdf(urdf_str, output_file_path):
    if not (output_file_path == "" or output_file_path is None):
        try:
            # Save the urdf data to a file
            with open(output_file_path, "w") as file:
                file.write(urdf_str)
                print("Saved urdf data to file: ", output_file_path)
        except:
            print("Error saving urdf data to file: ", output_file_path)
    else:
        print("ERROR: No output file path provided")
        
def _visualize_urdf(urdf_str):
    file_obj =  io.StringIO(urdf_str)
    urdf_model = yourdfpy.URDF.load(file_obj)
    
    # Validate the URDF model
    if urdf_model.validate():
        print("URDF model is valid")
    else:
        print("URDF model is not valid")
    # print("---------------------------------")
    
    # Show the URDF model
    urdf_model.show()

def _json_str_to_urdf(json_data):
    data = json.loads(json_data)
    
    # Create the root element of the URDF
    robot = Element('robot')
    robot.set('name', data['Name'])

    # Create an empty root link
    root_link = SubElement(robot, 'link', {'name': data['Name']})

    # Process each rigid body to create links and joints
    for body in data['RigidBodies']:
        # Extract file name without extension for link naming
        file_name = body['geometryFile'].split('/')[-1].split('.')[0]
        link_name = f"link_{file_name}_id_{body['id']}"

        # Create link element
        link = SubElement(robot, 'link', {'name': link_name})

        # Visual element
        visual = SubElement(link, 'visual')
        geometry_v = SubElement(visual, 'geometry')
        
        if "primitives/box.obj" in body['geometryFile']:
            box_v = SubElement(geometry_v, 'box')
            box_v.set('size', ' '.join(map(str, body['scale'])))
            
        else:
            mesh_v = SubElement(geometry_v, 'mesh')
            mesh_v.set('filename', f"file://{body['geometryFile']}")
            mesh_v.set('scale', ' '.join(map(str, body['scale'])))
    
        # Collision element
        collision = SubElement(link, 'collision')
        geometry_c = SubElement(collision, 'geometry')
        
        if "primitives/box.obj" in body['geometryFile']:
            box_c = SubElement(geometry_c, 'box')
            box_c.set('size', ' '.join(map(str, body['collisionObjectScale'])))
        
        else:
            mesh_c = SubElement(geometry_c, 'mesh')
            mesh_c.set('filename', f"file://{body['geometryFile']}")    
            mesh_c.set('scale', ' '.join(map(str, body['collisionObjectScale'])))
        


        # Fixed joint connecting this link to the root link
        joint = SubElement(robot, 'joint', {'name': f'joint_{link_name}', 'type': 'fixed'})
        parent = SubElement(joint, 'parent', {'link': data['Name']})
        child = SubElement(joint, 'child', {'link': link_name})
        origin = SubElement(joint, 'origin')
        origin.set('xyz', ' '.join(map(str, body['translation'])))
        angle = body['rotationAngle']
        axis = body['rotationAxis']
        q = np.array([
            np.cos(angle / 2),
            np.sin(angle / 2) * axis[0],
            np.sin(angle / 2) * axis[1],
            np.sin(angle / 2) * axis[2]
        ])
        
        roll = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
        s2 = 2*(q[0]*q[2] - q[3]*q[1])
        pitch = np.arcsin(np.clip(s2, -1.0, 1.0)) # Clipping to avoid NaN due to machine precision
        yaw = np.arctan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))
        
        # print("angle:", angle)
        # print("axis:", axis)
        # print("q:", q)
        # print(' '.join(map(str, [roll, pitch, yaw])))
        origin.set('rpy', ' '.join(map(str, [roll, pitch, yaw])))

        # Add other properties as metadata
        for key, value in body.items():
            if key not in ['id', 'geometryFile', 'translation', 'rotationAxis', 'rotationAngle', 'scale', 'collisionObjectScale']:
                meta = SubElement(link, key)
                meta.text = str(value)

    # Convert the XML tree to a pretty-printed string
    rough_string = tostring(robot, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    
    urdf_str = reparsed.toprettyxml(indent="  ")
    
    return urdf_str

def json_to_urdf(input_file_path,
                save_output=False, output_file_path=None,
                visualize=False):
    
    # Check if the input file exists
    if not os.path.exists(input_file_path):
        print("Input file does not exist.")
        return None
    
    # read the json file as a string
    try:
        with open(input_file_path, "r") as file:
            json_str = file.read()
    
        urdf_str = _json_str_to_urdf(json_str)
        
        if visualize:
            _visualize_urdf(urdf_str)
        
        if save_output:
            _save_urdf(urdf_str, output_file_path)
        
        return urdf_str
    
    except Exception as e:
        print("Error reading json file.")
        print(e)
        return None

def json_str_to_urdf(json_str, 
                     save_output=False, output_file_path=None,
                     visualize=False):
    
    urdf_str = _json_str_to_urdf(json_str)
    
    if visualize:
        _visualize_urdf(urdf_str)
    
    if save_output:
        _save_urdf(urdf_str, output_file_path)
            
    return urdf_str
