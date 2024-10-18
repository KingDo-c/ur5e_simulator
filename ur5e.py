import numpy as np
import open3d as o3d
import trimesh

# Define joint parameters (symbolic representation replaced with numerical values)
q1, q2, q3, q4, q5, q6 = 1, 1, 1, 1, 1, 1

# Link parameters
d0 = 0.089159  # meter
a1 = 0.425
a2 = 0.39225
d3 = 0.10915
d4 = 0.09465
d5 = 0.0823

def Mq(theta, axis):
    if axis == 'x':
        return np.array([[1, 0, 0, 0],
                         [0, np.cos(theta), -np.sin(theta), 0],
                         [0, np.sin(theta), np.cos(theta), 0],
                         [0, 0, 0, 1]])
    elif axis == 'y':
        return np.array([[np.cos(theta), 0, np.sin(theta), 0],
                         [0, 1, 0, 0],
                         [-np.sin(theta), 0, np.cos(theta), 0],
                         [0, 0, 0, 1]])
    elif axis == 'z':
        return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                         [np.sin(theta), np.cos(theta), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")

# Define transformation matrices
M_01 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, d0],
                 [0, 0, 0, 1]]) @ Mq(q1, 'z')

M_12 = np.array([[0, 0, 1, 0],
                 [1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]]) @ Mq(q2, 'z')

M_23 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, a1],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]) @ Mq(q3, 'z')

M_34 = np.array([[1, 0, 0, 0],
                 [0, 1, 0, a2],
                 [0, 0, 1, d3],
                 [0, 0, 0, 1]]) @ Mq(q4, 'z')

M_45 = np.array([[0, 1, 0, 0],
                 [0, 0, 1, d4],
                 [1, 0, 0, 0],
                 [0, 0, 0, 1]]) @ Mq(q5, 'z')

M_56 = np.array([[0, 0, 1, d5],
                 [1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]]) @ Mq(q6, 'z')
print(M_01)
print(M_12)
print(M_23)
print(M_34)
print(M_45)
print(M_56)


# Calculate full transformation from base to end-effector
T_matrices = [np.eye(4), 
              M_01 , 
              M_01 @ M_12 , 
              M_01 @ M_12 @ M_23 , 
              M_01 @ M_12 @ M_23 @ M_34 , 
              M_01 @ M_12 @ M_23 @ M_34 @ M_45 ,
              M_01 @ M_12 @ M_23 @ M_34 @ M_45 @ M_56 ]

# Extract positions for each frame
positions = np.array([T[:3, 3] for T in T_matrices])

# Create Open3D geometries for visualization
lines = []
points = positions.tolist()
for i in range(len(positions) - 1):
    lines.append([i, i + 1])

# Create line set for the robot links
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector([[0, 0, 0] for _ in range(len(lines))])

# Create coordinate frames for each joint
coordinate_frames = []
for T in T_matrices:
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    frame.transform(T)
    coordinate_frames.append(frame)

# Load STL files and apply transformations
meshes = []
stl_files = [
    'stl/Base_UR5_STEP.stl',    
    'stl/Link1_UR5_STEP.stl',   
    'stl/Link2_UR5_STEP.stl',   
    'stl/Link3_UR5_STEP.stl',   
    'stl/Link4_UR5_STEP.stl',   
    'stl/Link5_UR5_STEP.stl',   
    'stl/Link6_UR5_STEP.stl'   
]

for i, stl_file in enumerate(stl_files):
    try:
        mesh = trimesh.load_mesh(stl_file)
        vertices = np.array(mesh.vertices)

        # Apply transformation
        T = T_matrices[i]
        vertices_homogeneous = np.hstack((vertices, np.ones((vertices.shape[0], 1))))
        vertices_transformed = (T @ vertices_homogeneous.T).T[:, :3]

        # Convert to Open3D mesh
        mesh_o3d = o3d.geometry.TriangleMesh()
        mesh_o3d.vertices = o3d.utility.Vector3dVector(vertices_transformed)
        mesh_o3d.triangles = o3d.utility.Vector3iVector(np.array(mesh.faces))
        mesh_o3d.compute_vertex_normals()
        mesh_o3d.paint_uniform_color([0.8, 0.8, 1.0])

        meshes.append(mesh_o3d)
    except Exception as e:
        print(f"Error loading {stl_file}: {e}")

# Visualize the robot using Open3D
o3d.visualization.draw_geometries([line_set] + meshes + coordinate_frames)