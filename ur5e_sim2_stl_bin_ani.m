% Define symbolic variables for joint parameters
syms q1 q2 q3 q4 q5 q6

% Link parameters
d0 = 0.089159; % meter
a1 = 0.425;
a2 = 0.39225;
d3 = 0.10915;
d4 = 0.09465;
d5 = 0.0823;

% Forward Kinematics Transformation matrices
M_01 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d0;
        0 0 0 1];

M_12 = [cos(q1) -sin(q1) 0 0;
        sin(q1) cos(q1)  0 0;
        0       0        1 0;
        0       0        0 1];

M_23 = [cos(q2) -sin(q2) 0 a1;
        sin(q2) cos(q2)  0 0;
        0       0        1 0;
        0       0        0 1];

M_34 = [cos(q3) -sin(q3) 0 a2;
        sin(q3) cos(q3)  0 0;
        0       0        1 d3;
        0       0        0 1];

M_45 = [cos(q4) -sin(q4) 0 0;
        sin(q4) cos(q4)  0 0;
        0       0        1 d4;
        0       0        0 1];

M_56 = [cos(q5) -sin(q5) 0 0;
        sin(q5) cos(q5)  0 0;
        0       0        1 d5;
        0       0        0 1];

M_67 = [cos(q6) -sin(q6) 0 0;
        sin(q6) cos(q6)  0 0;
        0       0        1 0;
        0       0        0 1];

% Full transformation from base to end-effector
T06 = M_01 * M_12 * M_23 * M_34 * M_45 * M_56 * M_67;

% Create main figure and subplots
f = figure;
subplot(1, 2, 1); % Main plot on the left side
hold on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);
grid on;
title('UR5e Robot Moving to Desired Position');

subplot(1, 2, 2); % Subplot for sliders on the right side
axis off;

% Create sliders for desired end-effector position
x_slider = uicontrol('Parent', f, 'Style', 'slider', 'Min', -1, 'Max', 1, 'Value', 0.5, 'Units', 'normalized', 'Position', [0.7, 0.6, 0.2, 0.05], 'Tag', 'x_slider');
y_slider = uicontrol('Parent', f, 'Style', 'slider', 'Min', -1, 'Max', 1, 'Value', 0.2, 'Units', 'normalized', 'Position', [0.7, 0.5, 0.2, 0.05], 'Tag', 'y_slider');
z_slider = uicontrol('Parent', f, 'Style', 'slider', 'Min', -1, 'Max', 1, 'Value', 0.3, 'Units', 'normalized', 'Position', [0.7, 0.4, 0.2, 0.05], 'Tag', 'z_slider');
x_label = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.7, 0.65, 0.2, 0.05], 'String', 'X: 0.5', 'Tag', 'x_label');
y_label = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.7, 0.55, 0.2, 0.05], 'String', 'Y: 0.2', 'Tag', 'y_label');
z_label = uicontrol('Parent', f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.7, 0.45, 0.2, 0.05], 'String', 'Z: 0.3', 'Tag', 'z_label');

% Callback function to update plot based on slider values
function updatePlot(~, ~)
    % Redefine symbolic variables and transformation matrices inside the function
    syms q1 q2 q3 q4 q5 q6

    d0 = 0.089159; % meter
    a1 = 0.425;
    a2 = 0.39225;
    d3 = 0.10915;
    d4 = 0.09465;
    d5 = 0.0823;

    M_01 = [1 0 0 0;
            0 1 0 0;
            0 0 1 d0;
            0 0 0 1];

    M_12 = [cos(q1) -sin(q1) 0 0;
            sin(q1) cos(q1)  0 0;
            0       0        1 0;
            0       0        0 1];

    M_23 = [cos(q2) -sin(q2) 0 a1;
            sin(q2) cos(q2)  0 0;
            0       0        1 0;
            0       0        0 1];

    M_34 = [cos(q3) -sin(q3) 0 a2;
            sin(q3) cos(q3)  0 0;
            0       0        1 d3;
            0       0        0 1];

    M_45 = [cos(q4) -sin(q4) 0 0;
            sin(q4) cos(q4)  0 0;
            0       0        1 d4;
            0       0        0 1];

    M_56 = [cos(q5) -sin(q5) 0 0;
            sin(q5) cos(q5)  0 0;
            0       0        1 d5;
            0       0        0 1];

    M_67 = [cos(q6) -sin(q6) 0 0;
            sin(q6) cos(q6)  0 0;
            0       0        1 0;
            0       0        0 1];

    T06 = M_01 * M_12 * M_23 * M_34 * M_45 * M_56 * M_67;

    % Find sliders and labels
    x_slider = findobj('Tag', 'x_slider');
    y_slider = findobj('Tag', 'y_slider');
    z_slider = findobj('Tag', 'z_slider');
    x_label = findobj('Tag', 'x_label');
    y_label = findobj('Tag', 'y_label');
    z_label = findobj('Tag', 'z_label');

    xd = x_slider.Value;
    yd = y_slider.Value;
    zd = z_slider.Value;
    set(x_label, 'String', sprintf('X: %.2f', xd));
    set(y_label, 'String', sprintf('Y: %.2f', yd));
    set(z_label, 'String', sprintf('Z: %.2f', zd));

    % Solve for joint angles using inverse kinematics
    solutions = solve(T06(1,4) == xd, T06(2,4) == yd, T06(3,4) == zd, [q1, q2, q3, q4, q5, q6]);
    q1_sol = double(solutions.q1);
    q2_sol = double(solutions.q2);
    q3_sol = double(solutions.q3);
    q4_sol = double(solutions.q4);
    q5_sol = double(solutions.q5);
    q6_sol = double(solutions.q6);

    % Update the robot plot
    T01 = double(subs(M_01 * M_12, q1, q1_sol));
    T12 = double(subs(M_23, q2, q2_sol));
    T23 = double(subs(M_34, q3, q3_sol));
    T34 = double(subs(M_45, q4, q4_sol));
    T45 = double(subs(M_56, q5, q5_sol));
    T56 = double(subs(M_67, q6, q6_sol));

    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    T06 = T05 * T56;

    T_matrices = {eye(4), T01, T02, T03, T04, T05, T06};

    positions = zeros(7, 3);
    for i = 1:7
        T = T_matrices{i};
        positions(i, :) = T(1:3, 4)';
    end

    % Clear current plot and re-plot the robot
    subplot(1, 2, 1);
    cla;
    hold on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3);
    grid on;
    plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'o-', 'LineWidth', 2);
    title('UR5e Robot Moving to Desired Position');
    hold off;
end

% Set the callback for the sliders
x_slider.Callback = @updatePlot;
y_slider.Callback = @updatePlot;
z_slider.Callback = @updatePlot;

% Initial plot
updatePlot();