import numpy as np
import open3d as o3d
import trimesh
import time

# Forward Kinematics function
def calc_TF(q_values):
    d0, a1, a2, d3, d4, d5 = 0.089159, 0.425, 0.39225, 0.10915, 0.09465, 0.0823
    
    def Mq(theta, axis):
        if axis == 'z':
            return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                             [np.sin(theta), np.cos(theta), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        else:
            raise ValueError("Unsupported axis for this implementation")
    
    q_values = list(q_values) + [0] * (6 - len(q_values))
    q1, q2, q3, q4, q5, q6 = q_values

    # Define transformation matrices for each joint
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

    # Calculate full transformation from base to end-effector
    M_02 = M_01 @ M_12 
    M_03 = M_01 @ M_12 @ M_23 
    M_04 = M_01 @ M_12 @ M_23 @ M_34 
    M_05 = M_01 @ M_12 @ M_23 @ M_34 @ M_45
    M_06 = M_01 @ M_12 @ M_23 @ M_34 @ M_45 @ M_56
    return [M_01, M_02, M_03, M_04, M_05, M_06]

# Open3D visualization with STL, coordinate frames, and animation
def visualize_robot():
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Load STL files and create meshes, coordinate frames, and lines
    q_values = [1, 1, 1, 1, 1, 1]  # Fixed joint angles
    T_matrices = [np.eye(4)] + calc_TF(q_values)
    meshes = []
    stl_files = [
        'stl/Base_UR5_STEP_c.stl',
        'stl/Link1_UR5_STEP_c.stl',
        'stl/Link2_UR5_STEP_c.stl',
        'stl/Link3_UR5_STEP_c.stl',
        'stl/Link4_UR5_STEP_c.stl',
        'stl/Link5_UR5_STEP_c.stl',
        'stl/Link6_UR5_STEP_c.stl' 
    ]

    coordinate_frames = []
    lines = []
    points = [T[:3, 3] for T in T_matrices]
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
            vis.add_geometry(mesh_o3d)

            # Create coordinate frame for each joint
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            frame.transform(T)
            coordinate_frames.append(frame)
            vis.add_geometry(frame)

        except Exception as e:
            print(f"Error loading {stl_file}: {e}")

    # Create line set for the robot links
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(points) - 1)])
    line_set.colors = o3d.utility.Vector3dVector([[0, 0, 0] for _ in range(len(points) - 1)])
    vis.add_geometry(line_set)

    while True:
        # Fixed joint angles to prevent unnecessary rotation
        q_values = [1, 1, 1, 1, 1, 1]
        T_matrices = [np.eye(4)] + calc_TF(q_values)
        positions = np.array([T[:3, 3] for T in T_matrices])

        # Update coordinate frames
        for i, frame in enumerate(coordinate_frames):
            frame.translate(-frame.get_center(), relative=True)  # Reset translation
            frame.transform(T_matrices[i])  # Apply new transformation

        # Update line set points
        line_set.points = o3d.utility.Vector3dVector(positions)

        vis.update_geometry(line_set)
        for frame in coordinate_frames:
            vis.update_geometry(frame)

        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.05)

    vis.destroy_window()

# Run visualization with STL, coordinate frames, and animation
visualize_robot()