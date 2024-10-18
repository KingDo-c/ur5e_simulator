% UR5e 로봇 시뮬레이터 (실시간 제어 기능 및 STL 파일 로드 포함)

% 초기 조인트 각도 설정 (예: 모두 0으로 설정)
theta = [0, 0, 0, 0, 0, 0];

% 학습률 설정 (기본값)
learning_rate = 0.1;

% STL 파일 경로 설정
stl_files = {
    'stl/Base_UR5_STEP.stl',    
    'stl/Link1_UR5_STEP.stl',   
    'stl/Link2_UR5_STEP.stl',   
    'stl/Link3_UR5_STEP.stl',   
    'stl/Link4_UR5_STEP.stl',   
    'stl/Link5_UR5_STEP.stl',   
    'stl/Link6_UR5_STEP.stl'   
};

% 심볼릭 변수 정의
syms q1 q2 q3 q4 q5 q6

% 링크 파라미터 설정
d0 = 0.089159; % meter
a1 = 0.425;
a2 = 0.39225;
d3 = 0.10915;
d4 = 0.09465;
d5 = 0.0823;

% 변환 행렬 정의
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

% 실시간 제어를 위한 figure 설정
f = figure('Position', [100, 100, 1400, 900]);
subplot(1, 2, 1);
axis equal;
hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;
view(3);
xlim([-1.5, 1.5]);
ylim([-1.5, 1.5]);
zlim([0, 1.5]);

% 조인트 각도 슬라이더 UI 생성 (subplot에 포함되지 않도록 별도로 설정)
joint_texts = gobjects(1, 6);
joint_sliders = gobjects(1, 6);
for i = 1:6
    joint_texts(i) = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.75, 0.9 - (i-1)*0.07, 0.2, 0.05], 'String', sprintf('Joint %d Angle: %.2f', i, theta(i)), 'FontSize', 10);
    joint_sliders(i) = uicontrol('Parent', f, 'Style', 'slider', 'Min', -pi, 'Max', pi, 'Value', theta(i), 'Units', 'normalized', 'Position', [0.75, 0.87 - (i-1)*0.07, 0.2, 0.04], 'Tag', sprintf('joint_slider_%d', i), 'Callback', @(src, event) set(joint_texts(i), 'String', sprintf('Joint %d Angle: %.2f', i, get(src, 'Value'))));
end

% 링크들의 위치 계산 및 STL 파일 시각화 (실시간 업데이트)
while ishandle(f)
    % 슬라이더 값을 통해 조인트 각도 설정
    for i = 1:6
        theta(i) = get(joint_sliders(i), 'Value');
    end

    % 포워드 기구학 계산 및 STL 파일 시각화
    points = [0; 0; 0];
    T = eye(4);
    subplot(1, 2, 1);
    cla;
    axis equal;
    hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on;
    view(3);
    xlim([-1.5, 1.5]);
    ylim([-1.5, 1.5]);
    zlim([0, 1.5]);

    % 각 링크의 위치 계산 및 STL 파일 적용
    T01 = M_01 * Mq(theta(1));
    T12 = M_12 * Mq(theta(2));
    T23 = M_23 * Mq(theta(3));
    T34 = M_34 * Mq(theta(4));
    T45 = M_45 * Mq(theta(5));
    T56 = M_56 * Mq(theta(6));

    T_matrices = {eye(4), T01, T01 * T12, T01 * T12 * T23, T01 * T12 * T23 * T34, T01 * T12 * T23 * T34 * T45, T01 * T12 * T23 * T34 * T45 * T56};
    
    for i = 1:6
        T = T_matrices{i + 1};
        points(:, i+1) = T(1:3, 4);
        plot3([points(1, i), points(1, i+1)], [points(2, i), points(2, i+1)], [points(3, i), points(3, i+1)], 'bo-');

        % STL 파일 로드 및 변환 적용
        [f_stl, v, n] = readSTLBinary(stl_files{i});
        R = T(1:3, 1:3); % 회전 행렬 (3x3)
        t = T(1:3, 4);   % 변환 벡터 (3x1)
        v_transformed = (v * R') + repmat(t', size(v, 1), 1);
        patch('Faces', f_stl, 'Vertices', v_transformed, 'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
    end
    drawnow;
end

disp('UR5e 실시간 시뮬레이션 완료');

% STL 파일 읽기 함수 정의
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

function Mqmat = Mq(q)
    Mqmat = [cos(q) -sin(q) 0 0;
             sin(q) cos(q)  0 0;
             0      0       1 0;
             0      0       0 1];
end