#!/usr/bin/env python3
#
# Copyright (c) 2025 Paul-Otto Müller
#
# https://github.com/paulotto/exosim
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

import os
import sys

import bpy
import csv
import math

from mathutils import Matrix
#from sklearn.decomposition import PCA

sys.path.append('/home/paul/miniconda3/envs/Blender/lib/python3.11/site-packages')
# Automatically add the folder containing this script to sys.path
script_dir = os.path.dirname(bpy.data.texts["core.py"].filepath)
if script_dir not in sys.path:
    sys.path.append(script_dir)

import visualize_trajectory as vt


# def compute_coordinate_system(points: np.ndarray, origin_index: int = -1) -> np.ndarray:
#     """
#     Compute a coordinate system from three or more points in space using PCA, ensuring that the resulting
#     coordinate system is orthonormal.
#
#     :param points: A numpy array of shape (n, 3) where n is the number of points.
#     :param origin_index: The index of the point to be used as the origin. If -1, the geometrical center is used.
#     :return: A homogeneous transformation matrix (4x4) representing the computed coordinate system.
#     """
#     if points.shape[0] < 3:
#         raise ValueError("At least three points are required to compute a coordinate system.")
#
#     if origin_index == -1:
#         origin = np.mean(points, axis=0)
#     else:
#         origin = points[origin_index]
#
#     centered_points = points - origin
#
#     pca = PCA(n_components=3)
#     pca.fit(centered_points)
#
#     coordinate_system = pca.components_.T
#     coordinate_system = QualysisData._make_rotation_matrix_orthonormal(coordinate_system)
#
#     T = np.eye(4)
#     T[:3, :3] = coordinate_system
#     T[:3, 3] = origin
#
#     return T


def export_attachments(object_names, file_path, scaling_factor=1.0, rotation=Matrix.Identity(3)):
    """
    Exports the coordinates of vertex groups to a CSV file if the vertex group contains only one vertex and the group
    exists in exactly two objects with the same name.
    :param object_names: A list of Blender objects to process.
    :param file_path: Path to the CSV file to save the output.
    :param scaling_factor: A factor to scale the vertex coordinates.
    :param rotation: A 3x3 rotation matrix to apply to the scaled coordinates.
    :return: None
    """
    # Ensure all provided objects exist in the scene
    objects = [bpy.data.objects[name] for name in object_names if name in bpy.data.objects]

    if len(objects) != len(object_names):
        print("Error: One or more objects in the provided list could not be found.")
    else:
        # Dictionary to store vertex groups by name
        vertex_groups_dict = {}

        # Collect vertex groups and their corresponding vertices for each object
        for obj in objects:
            if obj.type != 'MESH':
                print(f"Warning: Object '{obj.name}' is not a mesh. Skipping.")
                continue

            # Ensure in Object Mode to access vertex data safely
            if bpy.context.mode != 'OBJECT':
                bpy.ops.object.mode_set(mode='OBJECT')

            # Iterate through vertex groups in the object
            for vg in obj.vertex_groups:
                # Get the vertices assigned to this group
                group_indices = [v.index for v in obj.data.vertices if vg.index in [g.group for g in v.groups]]

                # Ensure this vertex group contains only one vertex
                if len(group_indices) != 1:
                    print(
                        f"Warning: Vertex group '{vg.name}' in object '{obj.name}' doesn't have exactly one vertex. Skipping.")
                    continue

                vertex_index = group_indices[0]  # Get the vertex index
                vertex = obj.data.vertices[vertex_index]  # Get the vertex
                global_coords = obj.matrix_world @ vertex.co  # Get global coordinates

                # Scale the coordinates
                scaled_coords = global_coords * scaling_factor

                # Apply the transformation matrix
                transformed_coords = rotation @ scaled_coords

                # Add to dictionary with vertex group name as the key
                if vg.name not in vertex_groups_dict:
                    vertex_groups_dict[vg.name] = []

                vertex_groups_dict[vg.name].append((obj.name, transformed_coords))

        # Write results to a CSV file
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)

            # Write the header row
            writer.writerow(
                ["vertex_group", "object_name_1", "object_name_2", "x_1", "y_1", "z_1", "x_2", "y_2", "z_2", "norm"])

            # Process vertex groups with data from exactly two objects
            for vg_name, data in vertex_groups_dict.items():
                if len(data) == 2:  # Only consider vertex groups present in exactly two objects
                    obj1_name, coords1 = data[0]
                    obj2_name, coords2 = data[1]
                    
                    d = math.sqrt((coords2.x - coords1.x)**2 + (coords2.y - coords1.y)**2 + (coords2.z - coords1.z)**2)

                    # Write the row
                    writer.writerow([
                        vg_name,
                        obj1_name, obj2_name,
                        round(coords1.x, 6), round(coords1.y, 6), round(coords1.z, 6),
                        round(coords2.x, 6), round(coords2.y, 6), round(coords2.z, 6),
                        round(d, 6)
                    ])

                    # Print for confirmation
                    print(f"Vertex Group: {vg_name}")
                    print(f"  {obj1_name} -> ({coords1.x:.4f}, {coords1.y:.4f}, {coords1.z:.4f})")
                    print(f"  {obj2_name} -> ({coords2.x:.4f}, {coords2.y:.4f}, {coords2.z:.4f})")
                elif len(data) > 2:
                    print(f"Warning: Vertex group '{vg_name}' exists in more than two objects. Skipping.")

            print(f"Vertex group comparison data has been written to '{file_path}'.")


def export_vertex_groups_to_csv(object_names, file_path, scaling_factor=1.0, rotation=Matrix.Identity(3)):
    """
    Exports vertex group coordinates of objects to a CSV file if the vertex group contains only one vertex.
    :param object_names: A list of Blender objects to process.
    :param file_path: Path to the CSV file to save the output.
    :param scaling_factor: A factor to scale the vertex coordinates.
    :param rotation: A 3x3 rotation matrix to apply to the scaled coordinates.
    """
    # Ensure all provided objects exist in the scene
    objects = [bpy.data.objects[name] for name in object_names if name in bpy.data.objects]

    if len(objects) != len(object_names):
        print("Error: One or more objects in the provided list could not be found.")
    else:
        # Open the CSV file for writing
        with open(file_path, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Write the header row
            writer.writerow(['object_name', 'vertex_group', 'x', 'y', 'z'])

            # Iterate through the objects
            for obj in objects:
                # Ensure the object is a mesh
                if obj.type != 'MESH':
                    print(f"Skipping {obj.name}: Not a mesh object.")
                    continue

                # Switch to object mode and ensure the object has vertex groups
                bpy.context.view_layer.objects.active = obj
                bpy.ops.object.mode_set(mode='OBJECT')

                if not obj.vertex_groups:
                    print(f"Skipping {obj.name}: No vertex groups.")
                    continue

                # Access the mesh data
                mesh = obj.data

                # Iterate through vertex groups
                for vg in obj.vertex_groups:
                    # Collect vertex indices belonging to the vertex group
                    vertex_indices = [
                        i for i, v in enumerate(mesh.vertices)
                        if any(g.group == vg.index for g in v.groups)
                    ]

                    # Check if the vertex group has exactly one vertex
                    if len(vertex_indices) == 1:
                        vertex_index = vertex_indices[0]
                        vertex = mesh.vertices[vertex_index]
                        global_co = obj.matrix_world @ vertex.co

                        # Scale the vertex coordinates and apply the rotation
                        scaled_co = global_co * scaling_factor
                        rotated_co = rotation @ scaled_co

                        # Write to the CSV file
                        writer.writerow(
                            [obj.name, vg.name, round(rotated_co.x, 6), round(rotated_co.y, 6), round(rotated_co.z, 6)])
                    else:
                        print(
                            f"Skipping vertex group '{vg.name}' in object '{obj.name}': Contains {len(vertex_indices)} vertices.")

        print(f"Vertex group data successfully exported to {file_path}")


def main():
    # Provide a list of object names to search
    object_names = ["mandible", "skull", "maxilla", "hyoid", 
                    "tmj_disc_imprint.cube.left.bool.proc.cut", "tmj_disc_imprint.cube.right.bool.proc.cut"]  # Replace with the names of your objects

    # Get the directory of the currently opened .blend file
    # If the file hasn't been saved yet, it will default to Blender's temp directory
    project_directory = bpy.path.abspath("//")

    # Define the output CSV file path
    output_csv_file = project_directory + "attachments.csv"

    # Coordinates scaling factora
    co_scale = 1e-3

    # Define a transformation matrix (for rotation, scaling, or translation)
    transformation_matrix = Matrix.Rotation(math.radians(90), 3, 'X') @ Matrix.Rotation(math.radians(180), 3, 'Y')

    # Export vertex group coordinates to a CSV file
    export_attachments(object_names, output_csv_file, co_scale, transformation_matrix)

    # Alternatively, export all vertex groups with one vertex to a CSV file
    object_names_2 = ["tmj_plane_constr_left", "tmj_plane_constr_right"]
    output_csv_file_2 = project_directory + "vertex_group_coordinates.csv"
    export_vertex_groups_to_csv(object_names_2, output_csv_file_2, co_scale, transformation_matrix)


if __name__ == "__main__":
    # main()
    vt.main()
