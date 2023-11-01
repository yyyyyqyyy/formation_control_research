% Calc_Pose_Error : Calculate Tracking Error of agents


%     A2 ———————————————————— A0
%     |                       |
%     |           VC          |
%     |                       |
%     A1 ———————————————————— A3   
%   
%  

function [error_x_a, error_y_a, true_pos_a] = Calc_Pose_Error(pos_v, target_distance_a, target_angle_a, pos_a)

%         input:
%                 pos_v = [x_v, y_v, theta_v]: pose of virtual center in a 3x1 list 
%                 target_distance_a: Distance Constraint Between Agent and Virtual Center 
%                 target_angle_a: Angle Constraint Between Agent and Virtual Center (radian)
%                 pos_a = [x_a, y_a, theta_a]: pose of agent in a 3x1 list


%         return:
%                 error_x_a: Error in the x-direction for the Agent
%                 error_y_a: Error in the y-direction for the Agent
%                 true_pos_a: The true position for the Agent when there is no error


    % The true position for the Agent when there is no error
    x_a_real = pos_v(1) + target_distance_a * cos(target_angle_a + pos_v(3));
    y_a_real = pos_v(2) + target_distance_a * sin(target_angle_a + pos_v(3)); 

    % Calculate the position error
    error_x_a = x_a_real - pos_a(1);
    error_y_a = y_a_real - pos_a(2);

    % Store the real position
    true_pos_a = [x_a_real; y_a_real];
end