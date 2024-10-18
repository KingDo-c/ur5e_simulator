% UR5e 로봇 시뮬레이터 (실시간 제어 기능 및 역기구학을 위한 Pseudo Inverse 알고리즘 적용)

% DH 파라미터 설정
a = [0, -0.425, -0.392, 0, 0, 0]; % 링크 길이 (m)
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]; % 링크 오프셋 (m)
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]; % 링크의 트위스트 각도 (rad)

% 초기 조인트 각도 설정 (예: 모두 0으로 설정)
theta = [0, 0, 0, 0, 0, 0];

% 학습률 설정 (기본값)
learning_rate = 0.1;

% 정적 기구학 계산을 위한 변환 행렬 생성 함수
function T = DH_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

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

learning_rate_text = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.75, 0.45, 0.2, 0.05], 'String', 'Learning Rate: 0.1', 'FontSize', 10);
learning_rate_slider = uicontrol('Parent', f, 'Style', 'slider', 'Min', 0.01, 'Max', 1, 'Value', 0.1, 'Units', 'normalized', 'Position', [0.75, 0.4, 0.2, 0.04], 'Tag', 'learning_rate_slider', 'Callback', @(src, event) set(learning_rate_text, 'String', sprintf('Learning Rate: %.2f', get(src, 'Value'))));

% 현재 XYZ 및 조인트 값 표시 텍스트 추가
theta_text = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.75, 0.3, 0.2, 0.05], 'String', 'Joint Angles: [0, 0, 0, 0, 0, 0]', 'FontSize', 10);
position_text = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.75, 0.25, 0.2, 0.05], 'String', 'Position: [0, 0, 0]', 'FontSize', 10);

% 링크들의 위치 계산 및 그리기 (실시간 업데이트)
while ishandle(f)
    % 슬라이더 값을 통해 조인트 각도 설정
    for i = 1:6
        theta(i) = get(joint_sliders(i), 'Value');
    end
    learning_rate = get(learning_rate_slider, 'Value');

    % 포워드 기구학 계산 및 그리기
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
    for i = 1:6
        T_i = DH_transform(a(i), alpha(i), d(i), theta(i));
        T = T * T_i;
        points(:, i+1) = T(1:3, 4);
        plot3([points(1, i), points(1, i+1)], [points(2, i), points(2, i+1)], [points(3, i), points(3, i+1)], 'bo-');
    end
    drawnow;

    % 현재의 XYZ 값과 조인트 값 업데이트
    current_position = points(:, end);
    set(theta_text, 'String', sprintf('Joint Angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]', theta));
    set(position_text, 'String', sprintf('Position: [%.2f, %.2f, %.2f]', current_position));
end

disp('UR5e 실시간 시뮬레이션 완료');