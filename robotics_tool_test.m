% Load the UR5e URDF file and display the robot model
ur5e = loadrobot("universalUR5e");

% Show the UR5e model with the visual meshes
show(ur5e);
% 
% % Call the helper function "exampleHelperSelectWaypoints" to get selected
% % waypoints data
% selectedWaypoints = exampleHelperSelectWaypoints();
% 
% % Consider default waypoints set for further calculations if no waypoints are selected
% % in the privious section
% if isempty(selectedWaypoints)
%     selectedWaypoints =  [ -0.3186    0.4080    0.1514; -0.3350    0.4932    0.1720; -0.3514    0.5798    0.1928;
%         -0.3653    0.6683    0.2139; -0.3817    0.7500    0.2338; -0.3948    0.8153    0.2496;-0.3158    0.8263    0.2447; 
%         -0.2397    0.8329    0.2407; -0.1565    0.8346    0.2368;-0.0603    0.8419    0.2357;
%         0.0311    0.8442    0.2358; 0.1307    0.8383    0.2366; 0.3935    0.8144    0.2492;
%         0.3786    0.7400    0.2312; 0.3664    0.6682    0.2141; 0.3538    0.5981    0.1972;
%         0.3207    0.4120    0.1525; 0.2180    0.4132    0.1473; 0.1021    0.4186    0.1449;
%         -0.0568    0.4185    0.1441; -0.1718    0.4180    0.1466; -0.3128    0.4086    0.1512];
% end
% 
% % Setup inverse kinematics object
% ik = inverseKinematics('RigidBodyTree',ur5e);
% % Disable random restarts
% ik.SolverParameters.AllowRandomRestart = false;
% ikWeights = [1 1 1 1 1 1];
% 
% % Set robot position to first waypoint for orientation setting
% firstWaypoint = selectedWaypoints(1,:);
% orient = [0 0 -pi/3];
% tgtPose = trvec2tform(firstWaypoint) * eul2tform(orient); %target pose
% config = ik('dispenserEdge',double(tgtPose),ikWeights',q_home');
% ur5e.Configuration = config;
% ur5e.ShowMarker = true;
% rotate3d off;
% 
% % Reset the helper function before using it to avoid the potential issues with
% % the persistent variables
% exampleHelperSetOrientations([],[],[],[],[],'reset');
% 
% zoom reset;
% pan off;
% view(145,25);
% set(gcf,'KeyPressFcn',{@exampleHelperSetOrientations,selectedWaypoints,ur5e,ik});
% disp("Press Up-Down arrow key to rotate about the X-axis. Press Left-Right arrow key to rotate about the Y-axis.");
% disp("Press C to save orientation for current waypoint and press N to move to the next waypoint.");
% 
% % remove the keyboard callback
% set(gcf,'KeyPressFcn',[]);
% 
% % Move cobot to the home position
% ur5e.Configuration = q_home;
% 
% % Disable the marker from the tool body
% ur5_iRBT.ShowMarker = false;
% 
% % Get the desired waypoint and orientation data
% [finalWaypoints, finalOrientations]= exampleHelperGetFinalWaypointData();
% 
% % Consider the default waypoint and orientation data if no orientation
% % setting has been done for waypoints
% if isempty(finalOrientations) && isempty(finalWaypoints)
%     finalWaypoints = selectedWaypoints;
%     finalOrientations = repmat([0 0 -pi/3],size(finalWaypoints,1),1);
% end