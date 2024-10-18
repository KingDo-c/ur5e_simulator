import numpy as np
import open3d as o3d
import trimesh
import time
from scipy.optimize import minimize

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

# Forward Kinematics
def calc_TF(q_values):
    
    q_values = list(q_values) + [0] * (6 - len(q_values))
    q1, q2, q3, q4, q5, q6 = q_values

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

    M_06 = M_01 @ M_12 @ M_23 @ M_34 @ M_45 @ M_56
    return (M_01, M_12, M_23, M_34, M_45, M_56)

def calc_FK(q_values):
    Tmat = calc_Tmat(q_values)
    return Tmat[-1]

def calc_Tmat(q_values):
    M_01, M_12, M_23, M_34, M_45, M_56 = calc_TF(q_values)
         
    M_02 = M_01 @ M_12 
    M_03 = M_01 @ M_12 @ M_23 
    M_04 = M_01 @ M_12 @ M_23 @ M_34 
    M_05 = M_01 @ M_12 @ M_23 @ M_34 @ M_45
    M_06 = M_01 @ M_12 @ M_23 @ M_34 @ M_45 @ M_56
    return [M_01, M_02, M_03, M_04, M_05, M_06]

# Jacobian
def calc_J(q_values):
    epsilon = 1e-6 
    J = np.zeros((3, 6)) 
    
    M_06 = calc_FK(q_values)
    p_06 = M_06[:3, 3]
    
    for i in range(6):
        dq = np.copy(q_values)
        dq[i] += epsilon
        
        M_06_dq = calc_FK(dq)
        p_06_dq = M_06_dq[:3, 3]
        
        J[:, i] = (p_06_dq - p_06) / epsilon

    return J

def calc_IK(target_position, initial_guess, max_iter=500):
    q_values = np.array(initial_guess, dtype=float)
    for _ in range(max_iter):
        M_06 = calc_FK(q_values)
        p_06 = M_06[:3, 3]

        error = target_position - p_06

        if np.linalg.norm(error) < 1e-6:
            return q_values

        J = calc_J(q_values)

        J_pinv = np.linalg.pinv(J)
        q_values += J_pinv @ error

    print("IK no sol...")
    return None

##############################################

def update_visualization(q_values, line_set, coordinate_frames, meshes, stl_files):
    # Forward Kinematics
    T_matrices = [np.eye(4)] + calc_Tmat(q_values)

    # 각 프레임의 위치 업데이트
    positions = np.array([T[:3, 3] for T in T_matrices])
    line_set.points = o3d.utility.Vector3dVector(positions.tolist())

    # 좌표 프레임 업데이트
    for i, T in enumerate(T_matrices):
        coordinate_frames[i].translate(-np.array(coordinate_frames[i].get_center()))  
        coordinate_frames[i].transform(T)  

    # STL 메쉬 업데이트
    for i, stl_file in enumerate(stl_files):
        try:
            mesh = trimesh.load_mesh(stl_file)
            vertices = np.array(mesh.vertices)

            # 변환 적용
            T = T_matrices[i]
            vertices_homogeneous = np.hstack((vertices, np.ones((vertices.shape[0], 1))))
            vertices_transformed = (T @ vertices_homogeneous.T).T[:, :3]

            # Open3D 메쉬로 변환
            meshes[i].vertices = o3d.utility.Vector3dVector(vertices_transformed)
            meshes[i].compute_vertex_normals()
        except Exception as e:
            print(f"Error loading {stl_file}: {e}")

# Grid 생성 함수
def create_grid(size=5, step=0.5):
    lines = []
    for i in np.arange(-size, size + step, step):
        # x 방향 선들
        lines.append([[i, -size, 0], [i, size, 0]])
        # y 방향 선들
        lines.append([[-size, i, 0], [size, i, 0]])

    # 선 생성
    line_set = o3d.geometry.LineSet()
    line_points = np.array(lines).reshape((-1, 3))
    line_set.points = o3d.utility.Vector3dVector(line_points)

    line_indices = []
    for i in range(0, len(lines) * 2, 2):
        line_indices.append([i, i + 1])
    line_set.lines = o3d.utility.Vector2iVector(line_indices)

    line_set.colors = o3d.utility.Vector3dVector([[0.8, 0.8, 0.8] for _ in range(len(line_indices))])
    return line_set

# 초기 설정
q_values = [0, 0, 0, 0, 0, 0]  # 초기 관절 각도

# 로봇의 링크와 좌표 프레임 초기화
T_matrices = [np.eye(4)] + calc_Tmat(q_values)
positions = np.array([T[:3, 3] for T in T_matrices])
points = positions.tolist()
lines = [[i, i + 1] for i in range(len(positions) - 1)]

line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector([[0, 0, 0] for _ in range(len(lines))])

coordinate_frames = [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2) for _ in T_matrices]
for i, T in enumerate(T_matrices):
    coordinate_frames[i].transform(T)

meshes = []
stl_files = [
    'stl/Base_UR5_STEP_cc.stl',
    'stl/Link1_UR5_STEP_cc.stl',
    'stl/Link2_UR5_STEP_cc.stl',
    'stl/Link3_UR5_STEP_cc.stl',
    'stl/Link4_UR5_STEP_cc.stl',
    'stl/Link5_UR5_STEP_cc.stl',
    'stl/Link6_UR5_STEP_cc.stl'
]

for i, stl_file in enumerate(stl_files):
    try:
        mesh = trimesh.load_mesh(stl_file)
        vertices = np.array(mesh.vertices)

        # 변환 적용
        T = T_matrices[i]
        vertices_homogeneous = np.hstack((vertices, np.ones((vertices.shape[0], 1))))
        vertices_transformed = (T @ vertices_homogeneous.T).T[:, :3]

        # Open3D 메쉬로 변환
        mesh_o3d = o3d.geometry.TriangleMesh()
        mesh_o3d.vertices = o3d.utility.Vector3dVector(vertices_transformed)
        mesh_o3d.triangles = o3d.utility.Vector3iVector(np.array(mesh.faces))
        mesh_o3d.compute_vertex_normals()
        mesh_o3d.paint_uniform_color([0.8, 0.8, 1.0])

        meshes.append(mesh_o3d)
    except Exception as e:
        print(f"Error loading {stl_file}: {e}")

vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()
vis.add_geometry(line_set)
for mesh in meshes:
    vis.add_geometry(mesh)

# 그리드 추가
grid_line_set = create_grid(size=5, step=0.5)
vis.add_geometry(grid_line_set)

# Set camera view parameters
view_control = vis.get_view_control()
view_control.set_zoom(1.3)
view_control.set_front([1, 1, 1])
view_control.set_lookat([0, 0, 0])
view_control.set_up([0, 0, 1])

previous_q_values = q_values.copy()

# 초기 시각적 요소 업데이트
update_visualization(q_values, line_set, coordinate_frames, meshes, stl_files)
for frame in coordinate_frames:
    vis.update_geometry(frame)
for mesh in meshes:
    vis.update_geometry(mesh)
vis.poll_events()
vis.update_renderer()

try:
    while True:
        # 새로운 목표 위치 입력
        new_target = input("Enter target position (x y z) separated by spaces: ")
        try:
            target_position = [float(val) for val in new_target.split()]
            if len(target_position) != 3:
                raise ValueError("Please enter exactly 3 values for the target position.")
        except ValueError as e:
            print(e)
            continue

        # 초기 추정값을 이전 q_values로 설정
        initial_guess = previous_q_values
        q_values = calc_IK(target_position, initial_guess)
        print("calculated qval : ",q_values)
        end_effector_matrix = calc_FK(q_values)
        end_effector_position = end_effector_matrix[:3, 3]
        print("End-effector position: ", end_effector_position)
        
        if q_values is None:
            print("IK no sol...")
            continue

        # q_values가 변경되었을 때만 업데이트
        if not np.allclose(q_values, previous_q_values):
            current_q_values = np.array(previous_q_values, dtype=float)
            target_position = np.array(target_position)

            while True:
                # Calculate the forward kinematics for the current joint values
                M_06 = calc_FK(current_q_values)
                current_position = M_06[:3, 3]

                # Calculate the position error
                position_error = target_position - current_position

                # If the error is sufficiently small, break the loop
                if np.linalg.norm(position_error) < 1e-4:
                    break

                # Calculate the Jacobian for the current joint values
                J = calc_J(current_q_values)

                # Calculate the Jacobian pseudo-inverse
                J_pinv = np.linalg.pinv(J)

                # Update joint values using the Jacobian pseudo-inverse and position error
                delta_q = J_pinv @ position_error
                current_q_values += delta_q

                # Update visualization every few iterations
                update_visualization(current_q_values, line_set, coordinate_frames, meshes, stl_files)

                vis.update_geometry(line_set)
                for frame in coordinate_frames:
                    vis.update_geometry(frame)
                for mesh in meshes:
                    vis.update_geometry(mesh)
                vis.poll_events()
                vis.update_renderer()

            # Final update after reaching the desired position
            update_visualization(current_q_values, line_set, coordinate_frames, meshes, stl_files)
            vis.update_geometry(line_set)
            for frame in coordinate_frames:
                vis.update_geometry(frame)
            for mesh in meshes:
                vis.update_geometry(mesh)
            vis.poll_events()
            vis.update_renderer()

            previous_q_values = current_q_values.copy()
            
except KeyboardInterrupt:
    vis.destroy_window()