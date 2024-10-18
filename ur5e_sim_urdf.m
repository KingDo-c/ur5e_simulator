% 로봇 모델 불러오기
% ur5e = loadrobot("universalUR5e");
ur5e = importrobot('ur5e.urdf');

% 로봇 이름 설정
ur5e.DataFormat = 'row';  % 로봇 데이터 포맷 설정 (옵션: 'row', 'column', 'struct')
ur5e.Gravity = [0, 0, -9.81]; % 중력 설정 (옵션)

% 링크 간의 변환 행렬 설정하기
d1 = 0.1625; % meter
a2 = 0.425;
a3 = 0.3922;
d4 = 0.1333;
d5 = 0.0997;
d6 = 0.0996;

% 변환 행렬 정의
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

% 각 링크의 조인트에 변환 행렬 적용하기
setFixedTransform(ur5e.Bodies{2}.Joint, M_01); % 첫 번째 링크와 두 번째 링크 사이
setFixedTransform(ur5e.Bodies{3}.Joint, M_12); % 두 번째 링크와 세 번째 링크 사이
setFixedTransform(ur5e.Bodies{4}.Joint, M_23); % 세 번째 링크와 네 번째 링크 사이
setFixedTransform(ur5e.Bodies{5}.Joint, M_34); % 네 번째 링크와 다섯 번째 링크 사이
setFixedTransform(ur5e.Bodies{6}.Joint, M_45); % 다섯 번째 링크와 여섯 번째 링크 사이
setFixedTransform(ur5e.Bodies{7}.Joint, M_56); % 여섯 번째 링크와 마지막 링크 사이

% 기존 링크의 시각적 요소 덮어쓰기 (새로운 STL 파일 추가하기)
% 기본적으로 기존의 시각적 요소가 존재하더라도, addVisual로 새로운 시각적 요소를 추가하면 기존 요소를 덮어쓰는 효과가 있습니다.

% addVisual(ur5e.Bodies{1}, 'Mesh', 'ur5e_stl/Base_UR5_STEP.stl');     % Base 링크에 STL 추가
% addVisual(ur5e.Bodies{2}, 'Mesh', 'ur5e_stl/Link1_UR5_STEP.stl');    % Shoulder 링크에 STL 추가
% addVisual(ur5e.Bodies{3}, 'Mesh', 'ur5e_stl/Link2_UR5_STEP.stl');    % Upper arm 링크에 STL 추가
% addVisual(ur5e.Bodies{4}, 'Mesh', 'ur5e_stl/Link3_UR5_STEP.stl');    % Forearm 링크에 STL 추가
% addVisual(ur5e.Bodies{5}, 'Mesh', 'ur5e_stl/Link4_UR5_STEP.stl');    % First wrist 링크에 STL 추가
% addVisual(ur5e.Bodies{6}, 'Mesh', 'ur5e_stl/Link5_UR5_STEP.stl');    % Second wrist 링크에 STL 추가
% addVisual(ur5e.Bodies{7}, 'Mesh', 'ur5e_stl/Link6_UR5_STEP.stl');    % Third wrist 링크에 STL 추가

% 로봇 모델 시각화
show(ur5e, 'PreservePlot', false);
