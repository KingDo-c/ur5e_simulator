% UR5e 로봇 시뮬레이터 (실시간 제어 기능 및 역기구학을 위한 Pseudo Inverse 알고리즘 적용)

% DH 파라미터 설정
a = [0, -0.425, -0.392, 0, 0, 0]; % 링크 길이 (m)
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]; % 링크 오프셋 (m)
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]; % 링크의 트위스트 각도 (rad)

% 초기 조인트 각도 설정 (예: 모두 0으로 설정)
theta = [0, 0, 0, 0, 0, 0];

% 정적 기구학 계산을 위한 변환 행렬 생성 함수
function T = DH_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

% 실시간 제어를 위한 figure 설정
f = figure('Position', [100, 100, 1000, 800]);
subplot(1, 2, 1);
axis equal;
hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;
view(3);
xlim([-1.5, 1.5]);
ylim([-1.5, 1.5]);
zlim([0, 1.5]);

% 슬라이더 UI 생성 (subplot에 포함되지 않도록 별도로 설정)
x_text = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.75, 0.85, 0.2, 0.05], 'String', 'X Position: 0.5');
x_slider = uicontrol('Parent', f, 'Style', 'slider', 'Min', -0.5, 'Max', 0.5, 'Value', 0.5, 'Units', 'normalized', 'Position', [0.75, 0.8, 0.2, 0.05], 'Tag', 'x_slider', 'Callback', @(src, event) set(x_text, 'String', sprintf('X Position: %.2f', get(src, 'Value'))));

y_text = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.75, 0.75, 0.2, 0.05], 'String', 'Y Position: 0.2');
y_slider = uicontrol('Parent', f, 'Style', 'slider', 'Min', -0.5, 'Max', 0.5, 'Value', 0.2, 'Units', 'normalized', 'Position', [0.75, 0.7, 0.2, 0.05], 'Tag', 'y_slider', 'Callback', @(src, event) set(y_text, 'String', sprintf('Y Position: %.2f', get(src, 'Value'))));

z_text = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.75, 0.65, 0.2, 0.05], 'String', 'Z Position: 0.3');
z_slider = uicontrol('Parent', f, 'Style', 'slider', 'Min', 0, 'Max', 0.6, 'Value', 0.3, 'Units', 'normalized', 'Position', [0.75, 0.6, 0.2, 0.05], 'Tag', 'z_slider', 'Callback', @(src, event) set(z_text, 'String', sprintf('Z Position: %.2f', get(src, 'Value'))));

% 링크들의 위치 계산 및 그리기 (실시간 업데이트)
while ishandle(f)
    % 슬라이더 값을 통해 목표 위치 변화량 설정
    delta_x = [get(x_slider, 'Value'), get(y_slider, 'Value'), get(z_slider, 'Value')];

    % 자코비안 계산
    J = []; % 자코비안 초기화
    T = eye(4);
    z = [0; 0; 1];
    o_n = [0; 0; 0];
    for i = 1:6
        T_i = DH_transform(a(i), alpha(i), d(i), theta(i));
        T = T * T_i;
        o_i = T(1:3, 4);
        J_i = [cross(z, (o_n - o_i)); z];
        J = [J, J_i];
        z = T(1:3, 3);
    end

    % Pseudo Inverse를 사용하여 역기구학 계산
    J_pinv = pinv(J(1:3, :)); % 위치에 대한 자코비안의 Pseudo Inverse
    delta_theta = J_pinv * delta_x';
    
    % 이동을 100개의 스텝으로 나누어 천천히 이동
    num_steps = 100;
    for step = 1:num_steps
        theta = theta + (delta_theta' / num_steps);

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

        % 현재의 XYZ 값과 조인트 값 출력
        current_position = points(:, end);
        disp(['Step ', num2str(step), ': X = ', num2str(current_position(1)), ', Y = ', num2str(current_position(2)), ', Z = ', num2str(current_position(3))]);
        disp(['Joint Angles: ', num2str(theta)]);
    end
end

disp('UR5e 실시간 시뮬레이션 완료');