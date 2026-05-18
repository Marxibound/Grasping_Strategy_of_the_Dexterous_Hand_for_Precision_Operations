import os

def generate_urdf(obj_file):
    """
    Generate a URDF file for the given mesh file
    """
    obj_name = os.path.splitext(os.path.basename(obj_file))[0]
    urdf_content = f"""<?xml version="1.0"?>
<robot name="object">
  <link name="object">
    <visual>
      <origin xyz="0.0 0.0 0.0"/>
      <geometry>
        <mesh filename="{obj_file}" scale="1.00 1.00 1.00"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0"/>
      <geometry>
        <mesh filename="{obj_file}" scale="1.00 1.00 1.00"/>
      </geometry>
    </collision>
  </link>
</robot>
"""
    urdf_filename = os.path.join(os.path.dirname(obj_file), obj_name + ".urdf")
    with open(urdf_filename, 'w') as urdf_file:
        urdf_file.write(urdf_content)
    print(f"Generated URDF for {obj_file}: {urdf_filename}")

def process_directory(directory):
    """
    Process all mesh files in the directory and generate URDF files for them
    """
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith('.off'):
                obj_file = os.path.join(root, file)
                generate_urdf(obj_file)

# Usage example
directory = '/your_dataset_path/mesh'
process_directory(directory)