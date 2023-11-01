%  Get_Init_Poses : Compute the initial poses of agents and geometric constraint between the agent and the virtual center


%     A2 ———————————————————— A0
%     |                       |
%     |           VC          |
%     |                       |
%     A1 ———————————————————— A3   
%   
%     
function [pos_a0, pos_a1, pos_a2, pos_a3 ,target_angle_a0, target_distance_a0 ,target_angle_a1,target_distance_a1,target_angle_a2, target_distance_a2, target_angle_a3, target_distance_a3] ...
= Get_Init_Poses(center_x, center_y, center_theta, width, length)
    
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

    target_angle = atan((width / 2) / (length / 2));
    target_distance = (width / 2) / sin(target_angle);

    % Compute the initial poses and constraint of agent 0 
    target_angle_a0 = target_angle;
    target_distance_a0 = target_distance;
    x_a0 = center_x + cos(target_angle_a0 + center_theta) * target_distance_a0;
    y_a0 = center_y + sin(target_angle_a0 + center_theta) * target_distance_a0;
    theta_a0 = center_theta;

    % Store the initial pose of agent 0 in a 1x3 list
    pos_a0 = [x_a0, y_a0, theta_a0]';

    % Compute the initial poses and constraint of agent 1
    target_angle_a1 = target_angle;
    target_distance_a1 = -target_distance;
    x_a1 = center_x + cos(target_angle_a1 + center_theta) * target_distance_a1;
    y_a1 = center_y + sin(target_angle_a1 + center_theta) * target_distance_a1;
    theta_a1 = center_theta;

    % Store the initial pose of agent 1 in a  1x3 list
    pos_a1 = [x_a1, y_a1, theta_a1]';

    % Compute the initial poses and constraint of agent 2
    target_angle_a2 = -target_angle;
    target_distance_a2 = -target_distance;
    x_a2 = center_x + cos(target_angle_a2 + center_theta) * target_distance_a2;
    y_a2 = center_y + sin(target_angle_a2 + center_theta) * target_distance_a2;
    theta_a2 = center_theta;

    % Store the initial pose of agent 2 in a 1x3 list
    pos_a2 = [x_a2, y_a2, theta_a2]';

    % Compute the initial poses of and constraint agent 3
    target_angle_a3 = -target_angle;
    target_distance_a3 = target_distance;
    x_a3 = center_x + cos(target_angle_a3 + center_theta) * target_distance_a3;
    y_a3 = center_y + sin(target_angle_a3 + center_theta) * target_distance_a3;
    theta_a3 = center_theta;

    % Store the initial pose of agent 3 in a 1x3 list
    pos_a3 = [x_a3, y_a3, theta_a3]';


end