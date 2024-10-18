% Define symbolic variables for joint parameters
syms q1 q2 q3 q4 q5 q6

% Link parameters
d0 = 0.089159; % meter
a1 = 0.425;
a2 = 0.39225;
d3 = 0.10915;
d4 = 0.09465;
d5 = 0.0823;

% Transformation matrices
M_01 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d0;
        0 0 0 1];

M_12 = [0 0 1 0;
        1 0 0 0;
        0 1 0 0;
        0 0 0 1];

M_23 = [1 0 0 0;
        0 1 0 a1;
        0 0 1 0;
        0 0 0 1];

M_34 = [1 0 0 0;
        0 1 0 a2;
        0 0 1 d3;
        0 0 0 1];

M_45 = [0 1 0 0;
        0 0 1 d4;
        1 0 0 0;
        0 0 0 1];

M_56 = [0 0 1 d5;
        1 0 0 0;
        0 1 0 0;
        0 0 0 1];

% Transformation matrix with joint rotations (assuming q1, q2, ..., q6 = 0 for visualization)
T01 = M_01 * Mq(1); 
T12 = M_12 * Mq(-2); 
T23 = M_23 * Mq(2); 
T34 = M_34 * Mq(1); 
T45 = M_45 * Mq(-1); 
T56 = M_56 * Mq(0); 

% Full transformation from base to end-effector
T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;
T06 = T05 * T56;

% Extract transformation matrices for each frame
T_matrices = {eye(4), T01, T02, T03, T04, T05, T06};

% Plot the transformations
figure;
hold on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);
grid on;

% Plot frames
positions = zeros(7, 3);
quaternions = zeros(7, 4);

for i = 1:7
    T = T_matrices{i};
    positions(i, :) = T(1:3, 4)';
    rotation_matrix = T(1:3, 1:3);
    quaternions(i, :) = rotm2quat(rotation_matrix);
end

% Plot frame
plotTransforms(positions, quaternions, 'FrameSize', 0.1);

% Plot link
for i = 1:size(positions, 1) - 1
    plot3([positions(i, 1), positions(i+1, 1)], ...
          [positions(i, 2), positions(i+1, 2)], ...
          [positions(i, 3), positions(i+1, 3)], ...
          'k', 'LineWidth', 2); 
end

% Load STL files
stl_files = {
    'coordi2/ur5e_stl_bin/Base_UR5_STEP.stl',    
    'coordi2/ur5e_stl_bin/Link1_UR5_STEP.stl',   
    'coordi2/ur5e_stl_bin/Link2_UR5_STEP.stl',   
    'coordi2/ur5e_stl_bin/Link3_UR5_STEP.stl',   
    'coordi2/ur5e_stl_bin/Link4_UR5_STEP.stl',   
    'coordi2/ur5e_stl_bin/Link5_UR5_STEP.stl',   
    'coordi2/ur5e_stl_bin/Link6_UR5_STEP.stl'   
};

for i = 1:7
    % Load the STL file
    [f, v, n] = readSTLBinary(stl_files{i});  % 바이너리 STL 파일에서 faces, vertices, normals 읽기
    
    % Apply the transformation to the vertices
    T = T_matrices{i};
    R = T(1:3, 1:3); % 회전 행렬 (3x3)
    t = T(1:3, 4);   % 변환 벡터 (3x1)

    % Transform vertices: rotate and then translate
    v_transformed = (v * R'); % 모든 꼭짓점에 회전 적용 (v는 n x 3 행렬이므로 R의 전치를 사용해 곱셈)
    v_transformed = v_transformed + repmat(t', size(v, 1), 1); % 모든 꼭짓점에 변환 적용

    % Plot the STL part
    patch('Faces', f, 'Vertices', v_transformed, 'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
end


title('Visualization of UR5e Robot with STL Files');
hold off;

% Function definitions
function Mqmat = Mq(x)
    Mqmat = [cos(x) -sin(x) 0 0;
             sin(x) cos(x)  0 0;
             0      0       1 0;
             0      0       0 1];
end

function [f, v, n] = readSTLBinary(filename)
    % Reads a binary STL file
    fid = fopen(filename, 'rb');
    if fid == -1
        error('Could not open the file %s', filename);
    end
    
    % Skip the header (80 bytes)
    fread(fid, 80, 'uint8');
    
    % Read the number of triangles
    num_faces = fread(fid, 1, 'uint32');
    
    % Initialize arrays for faces, vertices, and normals
    f = zeros(num_faces, 3);
    v = zeros(num_faces * 3, 3);
    n = zeros(num_faces, 3);
    
    % Read the data for each triangle
    for i = 1:num_faces
        % Read normal vector (3 floats)
        n(i, :) = fread(fid, 3, 'float32');
        
        % Read vertices (3 vertices, each with 3 floats)
        v((i-1)*3+1:i*3, :) = fread(fid, [3, 3], 'float32')';
        
        % Read attribute byte count (2 bytes, ignored)
        fread(fid, 1, 'uint16');
        
        % Define face indices
        f(i, :) = [(i-1)*3+1, (i-1)*3+2, (i-1)*3+3];
    end
    
    fclose(fid);
end

camlight('headlight');
material('dull');

title('UR5e Simulator');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % TF EE
% T06 = M_01 * M_12 * M_23 * M_34 * M_45 * M_56 * M_67;
% 
% % Define the desired end-effector position
% xd = 0.5; % Desired x coordinate (in meters)
% yd = 0.2; % Desired y coordinate (in meters)
% zd = 0.3; % Desired z coordinate (in meters)
% 
% % IK
% solutions = solve(T06(1,4) == xd, T06(2,4) == yd, T06(3,4) == zd, [q1, q2, q3, q4, q5, q6]);
% 
% % Sol q
% q1_sol = double(solutions.q1);
% q2_sol = double(solutions.q2);
% q3_sol = double(solutions.q3);
% q4_sol = double(solutions.q4);
% q5_sol = double(solutions.q5);
% q6_sol = double(solutions.q6);
% 
% % Display Sol
% fprintf('Joint Angles to Reach Desired Position:\n');
% fprintf('q1: %.2f radians\n', q1_sol);
% fprintf('q2: %.2f radians\n', q2_sol);
% fprintf('q3: %.2f radians\n', q3_sol);
% fprintf('q4: %.2f radians\n', q4_sol);
% fprintf('q5: %.2f radians\n', q5_sol);
% fprintf('q6: %.2f radians\n', q6_sol);
% 
% % Visualization
% figure;
% hold on;
% axis equal;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% view(3);
% grid on;
% 
% % Plot robot
% T01 = double(subs(M_01 * M_12, q1, q1_sol));
% T02 = double(subs(T01 * M_23, q2, q2_sol));
% T03 = double(subs(T02 * M_34, q3, q3_sol));
% T04 = double(subs(T03 * M_45, q4, q4_sol));
% T05 = double(subs(T04 * M_56, q5, q5_sol));
% T06 = double(subs(T05 * M_67, q6, q6_sol));
% 
% T_matrices = {eye(4), T01, T02, T03, T04, T05, T06};
% 
% positions = zeros(7, 3);
% for i = 1:7
%     T = T_matrices{i};
%     positions(i, :) = T(1:3, 4)';
% end
% 
% % Plot frames
% plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'o-', 'LineWidth', 2);
% title('UR5e Simulator');
% hold off;
