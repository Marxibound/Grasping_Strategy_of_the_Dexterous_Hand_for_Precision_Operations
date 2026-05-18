import os
import numpy as np
import trimesh
import pickle

def find_mesh_files(directory):
    """
    Find all .off and .ply mesh files in the specified directory
    """
    mesh_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.off') or file.endswith('.ply'):
                mesh_files.append(os.path.join(root, file))
    return mesh_files

def load_point_cloud_with_normals(mesh_file):
    """
    Load a mesh file (.off or .ply) and generate point cloud data with normals
    """
    mesh = trimesh.load(mesh_file)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"File {mesh_file} is not a valid mesh.")
    
    # Sample 10240 points from the mesh
    points, face_indices = mesh.sample(10240, return_index=True)
    
    # Get normals corresponding to the sampled points
    normals = mesh.face_normals[face_indices]
    
    # Combine points and normals
    point_cloud_with_normals = np.hstack((points, normals))
    return point_cloud_with_normals

def save_point_cloud_data_with_normals(mesh_files, output_file):
    """
    Save point cloud data with normals from all mesh files to a single file
    """
    data = {}
    for mesh_file in mesh_files:
        # Load point cloud and normal data
        point_cloud_with_normals = load_point_cloud_with_normals(mesh_file)
        
        # Use the filename as the key in the dictionary
        file_name = os.path.splitext(os.path.basename(mesh_file))[0]
        data[file_name] = point_cloud_with_normals
    
    # Serialize and save the data using pickle
    with open(output_file, 'wb') as f:
        pickle.dump(data, f)

def process_directory_with_normals(directory, output_file):
    """
    Process all mesh files in the directory, generate point clouds with normals and save them
    """
    # Find all mesh files
    mesh_files = find_mesh_files(directory)
    
    # Save point cloud data with normals
    save_point_cloud_data_with_normals(mesh_files, output_file)

# Usage example
directory = '/your_dataset_path/mesh'  # Replace with your folder path
output_file = '/your_dataset_path/object_pcds_nors.pkl'  # Output file path
process_directory_with_normals(directory, output_file)

print(f"Point cloud data with normals has been saved to {output_file}")