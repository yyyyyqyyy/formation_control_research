
%% 
% build map
clear all;
load('mapmatrix.mat');
map = binaryOccupancyMap(mapmatrix,1);
% show(map)
%% 
% define Object to describe the kinematic characteristics of the Agent and virtual center

%     A2 ———————————————————— A0
%     |                       |
%     |           VC          |
%     |                       |
%     A1 ———————————————————— A3   
% 

% Creation of Agents and Virtual Center
center = differentialDriveKinematics("WheelRadius",0.1525,"TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
agent_0 = differentialDriveKinematics("WheelRadius",0.1525,"TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
agent_1 = differentialDriveKinematics("WheelRadius",0.1525,"TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
agent_2 = differentialDriveKinematics("WheelRadius",0.1525,"TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
agent_3 = differentialDriveKinematics("WheelRadius",0.1525,"TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");

%inflated map
mapInflated = copy(map);
inflate(mapInflated, center.TrackWidth/2);
show(mapInflated)

%% 
% Define the pose of the virtual center
center_x = 5.0;  
center_y = 10.0;
center_theta = 0.0;

% width (int):  shortest side of the palette
% length (int): longest side of the palette
width = 2* sqrt(2);
length = 2* sqrt(2);


%% Astar path planing

planner = plannerAStarGrid(map);
startLocation = [center_x center_y];
endLocation = [16.0 6.0];
path = plan(planner,startLocation,endLocation);

%% define controller
% controller = controllerPurePursuit;

% The virtual center moves at a speed of 0.3 meters per second.
controller = controllerPurePursuit('DesiredLinearVelocity', 0.3);
release(controller);
controller.Waypoints = path;

%start and end point of path
vc_InitialLocation = path(1,:);  
vc_Goal = path(end,:);
vc_initialOrientation = center_theta;  

%initial position of the virtual center
pos_v = [vc_InitialLocation vc_initialOrientation]';  

%Calculate the distance to the target position
vc_distanceToGoal = norm(vc_InitialLocation - vc_Goal); 
goalRadius = 0.3;

% sampleTime: The time interval for measurement, computation, and control during discrete-time steps
sampleTime = 0.1;

% Define the refresh rate for visualization
vizRate = rateControl(1/sampleTime);
reset(vizRate);

frameSize = center.TrackWidth/0.8;
%% 

% Get_Init_Poses : Compute the initial poses of agents and geometric constraint between the agent and the virtual center

%         input:
%                 center_x (float): virtual center x 
%                 center_y (float): virtual center y 
%                 center_theta (float): virtual center theta (radian)
%                 width (int):  shortest side of the palette
%                 length (int): longest side of the palette

%         return:
%                 pos_a1 = [x_a1, y_a1, theta_a1]    pose of agent 1 in a 3x1 list
%                 ...
%                 pos_a4 = [x_a4, y_a4, theta_a4]    pose of agent 4 in a 3x1 list
%                 target_angle_a0 (float): Angle Constraint Between Agent 0 and Virtual Center (radian)
%                 target_distance_a0 (int): Distance Constraint Between Agent 0 and Virtual Center
%                 ...
%                 target_angle_a3 (float): Angle Constraint Between Agent 3 and Virtual Center (radian)
%                 target_distance_a3 (int): Distance Constraint Between Agent 3 and Virtual Center

 [pos_a0, pos_a1, pos_a2, pos_a3 ,target_angle_a0, target_distance_a0 ,...
     target_angle_a1, target_distance_a1, target_angle_a2, target_distance_a2, target_angle_a3, target_distance_a3]...
= Get_Init_Poses(center_x, center_y, center_theta, width, length);

%% 
% d, k1, k2 : Parameters in the control law
k1 = 0.7;
k2 = 0.7;
d = 0.05;
%% 

pos_v_s = [];% Store all poses of virtual center

pos_a0_s = [];% Store all poses of Agent 0
pos_a1_s = [];% Store all poses of Agent 1
pos_a2_s = [];% Store all poses of Agent 2
pos_a3_s = [];% Store all poses of Agent 3

% Store all Error of Agent 0
error_x_a0_s= []; 
error_y_a0_s= [];
true_pos_a0 = [];
% Store all Error of Agent 1
error_x_a1_s= [];
error_y_a1_s= [];
true_pos_a1 = [];
% Store all Error of Agent 2
error_x_a2_s= [];
error_y_a2_s= [];
true_pos_a2 = [];
% Store all Error of Agent 3
error_x_a3_s= [];
error_y_a3_s= [];
true_pos_a3 = [];

% i =1;

%%  save as video
% 
videoFileName = 'case_1.avi';
videoWriter = VideoWriter(videoFileName, 'Motion JPEG AVI');
open(videoWriter);
%% 

while ((vc_distanceToGoal > goalRadius) )
% while (i==1)

    % Calc_Pose_Error : Calculate Tracking Error of agents

    %         input:
    %                 center_x (float): virtual center x 
    %                 center_y (float): virtual center y 
    %                 center_theta (float): virtual center theta (radian)
    %                 width (int):  shortest side of the palette
    %                 length (int): longest side of the palette
    
    %         return:
    %                 pos_a1 = [x_a1, y_a1, theta_a1]    pose of agent 1 in a 3x1 list
    %                 ...
    %                 pos_a4 = [x_a4, y_a4, theta_a4]    pose of agent 4 in a 3x1 list
    %                 target_angle_a0 (float): Angle Constraint Between Agent 0 and Virtual Center (radian)
    %                 target_distance_a0 (int): Distance Constraint Between Agent 0 and Virtual Center
    %                 ...
    %                 target_angle_a3 (float): Angle Constraint Between Agent 3 and Virtual Center (radian)
    %                 target_distance_a3 (int): Distance Constraint Between Agent 3 and Virtual Center

    [error_x_a0, error_y_a0, true_pos_a0] = Calc_Pose_Error(pos_v, target_distance_a0, target_angle_a0, pos_a0);

    % Store all Error of Agent 0
    error_x_a0_s = [error_x_a0_s; error_x_a0];
    error_y_a0_s = [error_y_a0_s; error_y_a0];

    [error_x_a1, error_y_a1, true_pos_a1] = Calc_Pose_Error(pos_v, target_distance_a1, target_angle_a1, pos_a1);

    % Store all Error of Agent 1
    error_x_a1_s = [error_x_a1_s; error_x_a1];
    error_y_a1_s = [error_y_a1_s; error_y_a1];


    [error_x_a2, error_y_a2, true_pos_a2] = Calc_Pose_Error(pos_v, target_distance_a2, target_angle_a2, pos_a2);

    % Store all Error of Agent 2
    error_x_a2_s = [error_x_a2_s; error_x_a2];
    error_y_a2_s = [error_y_a2_s; error_y_a2];

    [error_x_a3, error_y_a3, true_pos_a3] = Calc_Pose_Error(pos_v, target_distance_a3, target_angle_a3, pos_a3);

    % Store all Error of Agent 3
    error_x_a3_s = [error_x_a3_s; error_x_a3];
    error_y_a3_s = [error_y_a3_s; error_y_a3];

%% 
    % Get the virtual center's velocity using controller inputs  
    [v, omega] = controller(pos_v);

    % Get the virtual center's in the xy direction and angular velocit
    vel_v = derivative(center, pos_v, [v omega]);

    % Update the current pose of virtual center
    pos_v = pos_v + vel_v * sampleTime; 

    % Re-compute the distance to the goal
    vc_distanceToGoal = norm(pos_v(1:2) - vc_Goal(:));

%% 
    % updateAgentPosition :Based on the previous position of the agent and the position of the virtual center, 
    % calculate the agent's new position and velocity.

    %         input:
    %                 pos_a = [x_a, y_a, theta_a]: previous position of the agent in a 3x1 list
    %                 target_distance_a: Distance Constraint Between Agent and Virtual Center 
    %                 target_angle_a: Angle Constraint Between Agent and Virtual Center (radian)
    %                 d, k1, k2 : Parameters in the control law
    %                 sampleTime: The time interval for measurement, computation, and control during discrete-time steps
    %                 pos_v = [x_v, y_v, theta_v]: pose of virtual center in a 3x1 list
    %                 vel_v = [vel_v_x, vel_v_y, vel_v_w]' :velocity in the xy direction and angular velocity of the virtual center  
    %                 agent : Object describing the kinematic characteristics of the Agent
    
    %         return:
    %                 pos_a = [x_a, y_a, theta_a]: new position of the agent
    %                 v_a: velocity of the agent
    %                 w_a: angular velocity of the agent

    [pos_a0, v_a0, w_a0] = updateAgentPosition(pos_a0, pos_v, target_distance_a0, target_angle_a0, d, k1, k2, sampleTime, vel_v, agent_0);
    [pos_a1, v_a1, w_a1] = updateAgentPosition(pos_a1, pos_v, target_distance_a1, target_angle_a1, d, k1, k2, sampleTime, vel_v, agent_1);
    [pos_a2, v_a2, w_a2] = updateAgentPosition(pos_a2, pos_v, target_distance_a2, target_angle_a2, d, k1, k2, sampleTime, vel_v, agent_2);
    [pos_a3, v_a3, w_a3] = updateAgentPosition(pos_a3, pos_v, target_distance_a3, target_angle_a3, d, k1, k2, sampleTime, vel_v, agent_3);

    % Store all poses of virtual center
    pos_v_s=  [pos_v_s; pos_v'];

    % Store all poses of Agents
    pos_a0_s=  [pos_a0_s; pos_a0'];
    pos_a1_s=  [pos_a1_s; pos_a1'];
    pos_a2_s=  [pos_a2_s; pos_a2'];
    pos_a3_s=  [pos_a3_s; pos_a3'];
    
    %% 
    % Update the plot
    hold off
    show(map);
    hold all

    % Plot the target path given by path planner
    plot(path(:, 1), path(:, 2), "k--d", 'DisplayName', 'target path');
    
    % Plot the path of the Agents and Virtual Center as a set of transforms
    plotTrVec1 = [pos_v(1:2); 0];
    plotRot1 = axang2quat([0 0 1 pos_v(3)]);
    plotTransforms(plotTrVec1', plotRot1, 'MeshFilePath', 'multirotor.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize*0.8);

    plotTrVec2 = [pos_a0(1:2); 0];
    plotRot2 = axang2quat([0 0 1 pos_a0(3)]);
    plotTransforms(plotTrVec2', plotRot2, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);
    
    plotTrVec3 = [pos_a1(1:2); 0];
    plotRot3 = axang2quat([0 0 1 pos_a1(3)]);
    plotTransforms(plotTrVec3', plotRot3, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);

    plotTrVec4 = [pos_a2(1:2); 0];
    plotRot4 = axang2quat([0 0 1 pos_a2(3)]);
    plotTransforms(plotTrVec4', plotRot4, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);

    plotTrVec5 = [pos_a3(1:2); 0];
    plotRot5 = axang2quat([0 0 1 pos_a3(3)]);
    plotTransforms(plotTrVec5', plotRot5, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);
    
    % show labels
    legend('Location', 'Best');
    light;
    xlim([0 27])
    ylim([0 17])

   
    % Plot trajectory
    plot(pos_v_s(:, 1), pos_v_s(:, 2), 'r-', 'DisplayName', 'actual path');
    plot(pos_a0_s(:, 1), pos_a0_s(:, 2), 'b-', 'DisplayName', 'agent 0');
    plot(pos_a1_s(:, 1), pos_a1_s(:, 2), 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'agent 1');
    plot(pos_a2_s(:, 1), pos_a2_s(:, 2), 'm-','DisplayName', 'agent 2');
    plot(pos_a3_s(:, 1), pos_a3_s(:, 2), 'g-' ,'DisplayName', 'agent 3');


    line([pos_a0(1, :), pos_a3(1, :)], [pos_a0(2, :), pos_a3(2, :)], 'Color', 'k', 'HandleVisibility', 'off');
    line([pos_a2(1, :), pos_a1(1, :)], [pos_a2(2, :), pos_a1(2, :)], 'Color', 'k', 'HandleVisibility', 'off');
    line([pos_a0(1, :), pos_a2(1, :)], [pos_a0(2, :), pos_a2(2, :)], 'Color', 'k', 'HandleVisibility', 'off');
    line([pos_a1(1, :), pos_a3(1, :)], [pos_a1(2, :), pos_a3(2, :)], 'Color', 'k', 'HandleVisibility', 'off');


    waitfor(vizRate);

% save as video
    frame = getframe(gcf);
    writeVideo(videoWriter, frame);
%     
end
close(videoWriter);