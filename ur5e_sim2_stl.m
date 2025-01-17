% Define symbolic variables for joint parameters
syms q1 q2 q3 q4 q5 q6

% Link parameters
d1 = 0.1625; % meter
a2 = 0.425;
a3 = 0.3922;
d4 = 0.1333;
d5 = 0.0997;
d6 = 0.0996;

% Transformation matrices
M_01 = [0 0 1 0;
        1 0 0 0;
        0 1 0 d1;
        0 0 0 1];
M_12 = [1 0 0 0;
        0 1 0 a2;
        0 0 1 0;
        0 0 0 1];
M_23 = [1 0 0 0;
        0 1 0 a3;
        0 0 1 0;
        0 0 0 1];
M_34 = [0 1 0 0;
        0 0 1 0;
        1 0 0 d4;
        0 0 0 1];
M_45 = [0 0 1 0;
        1 0 0 0;
        0 1 0 d5;
        0 0 0 1];
M_56 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d6;
        0 0 0 1];

% Transformation matrix with joint rotations (assuming q1, q2, ..., q6 = 0 for visualization)
T01 = M_01 * Mq(1); 
T12 = M_12 * Mq(-2); 
T23 = M_23 * Mq(0); 
T34 = M_34 * Mq(0); 
T45 = M_45 * Mq(0); 
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
    'ur5e_stl/Base_UR5_STEP.stl',    
    'ur5e_stl/Link1_UR5_STEP.stl',   
    'ur5e_stl/Link2_UR5_STEP.stl',   
    'ur5e_stl/Link3_UR5_STEP.stl',   
    'ur5e_stl/Link4_UR5_STEP.stl',   
    'ur5e_stl/Link5_UR5_STEP.stl',   
    'ur5e_stl/Link6_UR5_STEP.stl'   
};

for i = 1:7
    % Load the STL file
    [f, v] = readAsciiSTL(stl_files{i});
    
    T = T_matrices{i};
    R = T(1:3, 1:3); % 회전 행렬 (3x3)
    t = T(1:3, 4);   % 변환 벡터 (3x1)
    
    v_transformed = (R * v')'; 
    v_transformed = v_transformed + repmat(t', size(v, 1), 1);

    patch('Faces', f, 'Vertices', v_transformed, 'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
end

camlight('headlight');
material('dull');

title('Visualization of UR5e Robot with STL Files');
hold off;

function Mqmat = Mq(x)
    Mqmat = [cos(x) -sin(x) 0 0;
             sin(x) cos(x)  0 0;
             0      0       1 0;
             0      0       0 1];
end

function [faces, vertices] = readAsciiSTL(filename)
    % This function reads an ASCII STL file and returns the faces and vertices.
    fid = fopen(filename, 'r'); 
    if fid == -1
        error('Cannot open the STL file');
    end
    
    vertices = [];
    faces = [];
    vertexIndex = 0;
    
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        
        if startsWith(line, 'vertex')
            vertex = sscanf(line, 'vertex %f %f %f');
            vertices = [vertices; vertex'];
            vertexIndex = vertexIndex + 1;
            
            if mod(vertexIndex, 3) == 0
                faces = [faces; vertexIndex-2, vertexIndex-1, vertexIndex];
            end
        end
    end
    
    fclose(fid);
end
                      