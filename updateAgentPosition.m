
% updateAgentPosition :This function employs control theory to maintain a fixed geometric constraint between the agent and the virtual center 
% and computes the velocity and position of the agent.
% Based on the previous position of the agent and the position of the virtual center, calculate the agent's new position and velocity.

function [pos_a, v_a, w_a] = updateAgentPosition(pos_a, pos_v, target_distance_a, target_angle_a, d, k1, k2, sampleTime, vel_v, agent)
    
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

    % Extract agent pose components
    x_a = pos_a(1);
    y_a = pos_a(2);
    th_a = pos_a(3);

    % Extract agentvirtual center pose components
    x_v = pos_v(1);
    y_v = pos_v(2);
    th_v = pos_v(3);


    % ltx, lty : In the virtual center's coordinate system, the actual Agent position coordinates are ltx, lty
    ltx = (x_a + d * cos(th_a) - x_v) * cos(th_v) + (y_a + d * sin(th_a) - y_v) * sin(th_v);
    lty = (y_a + d * sin(th_a) - y_v) * cos(th_v) - (x_a + d * cos(th_a) - x_v) * sin(th_v);

    % lzx, lzy : In the virtual center's coordinate system, the desired Agent position coordinates are lzx, lzy
    lzx = target_distance_a * cos(target_angle_a);
    lzy = target_distance_a * sin(target_angle_a);

    % Calculate e_th, e_x, e_y
    e_th = th_a - th_v;
    e_x = lzx - ltx;
    e_y = lzy - lty;

    % Calculate the linear velocity and angular velocity of the virtual center.
    v_v = sqrt(vel_v(1).^2 + vel_v(2).^2);
    w_v = vel_v(3);

    % Calculate the linear velocity and angular velocity of the Agent.
    v_a = (k1 * e_x + v_v - lty * w_v) * cos(e_th) + (k2 * e_y + ltx * w_v) * sin(e_th);
    w_a = ((-k1 * e_x - v_v + lty * w_v) * sin(e_th) - (-k2 * e_y - ltx * w_v) * cos(e_th)) / d;

    % Calculate new position of the Agent.
    vel_a = derivative(agent, pos_a, [v_a w_a]);
    pos_a = pos_a + vel_a * sampleTime;
end