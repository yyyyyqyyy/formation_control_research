  
% plotMultiAgentPose : Visualization of simulation results.

function plotMultiAgentPose(pos_v_s, pos_a0_s, pos_a1_s, pos_a2_s, pos_a3_s, pos_v ,pos_a0, pos_a1, pos_a2 ,pos_a3, map, frameSize)  

%         input:
%                 pos_v_s ： Store all poses of virtual center
%                 pos_a0_s ： Store all poses of agent 0
%                 pos_a3_s ： Store all poses of agent 3

%                 pos_a0 = [x_a, y_a, theta_a]: position of the agent 0 in a 3x1 list
%                 pos_a3 = [x_a, y_a, theta_a]: position of the agent 3 in a 3x1 list
%                 map: grid map
%                 frameSize: used to specify the size of the objects to be drawn

    hold off
    show(map);
    hold all

    % Plot the pose of the Agents and Virtual Center as a set of transforms
    
    % Plot the pose of the Virtual Center
    plotTrVec1 = [pos_v(1:2); 0];
    plotRot1 = axang2quat([0 0 1 pos_v(3)]);
    plotTransforms(plotTrVec1', plotRot1, 'MeshFilePath', 'multirotor.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize*0.8);

    % Plot the pose of the Agents 0
    plotTrVec2 = [pos_a0(1:2); 0];
    plotRot2 = axang2quat([0 0 1 pos_a0(3)]);
    plotTransforms(plotTrVec2', plotRot2, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);
    
    % Plot the pose of the Agents 1
    plotTrVec3 = [pos_a1(1:2); 0];
    plotRot3 = axang2quat([0 0 1 pos_a1(3)]);
    plotTransforms(plotTrVec3', plotRot3, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);

    % Plot the pose of the Agents 2
    plotTrVec4 = [pos_a2(1:2); 0];
    plotRot4 = axang2quat([0 0 1 pos_a2(3)]);
    plotTransforms(plotTrVec4', plotRot4, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);
    
    % Plot the pose of the Agents 3
    plotTrVec5 = [pos_a3(1:2); 0];
    plotRot5 = axang2quat([0 0 1 pos_a3(3)]);
    plotTransforms(plotTrVec5', plotRot5, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);
    
    % show labels
    legend('Location', 'Best');
    light;
    xlim([0 27])
    ylim([0 17])

   
    % Plot trajectory of the Virtual Center
    plot(pos_v_s(:, 1), pos_v_s(:, 2), 'r-', 'DisplayName', 'actual path');
    
    % Plot trajectory of the Agents
    plot(pos_a0_s(:, 1), pos_a0_s(:, 2), 'b-', 'DisplayName', 'agent 0');
    plot(pos_a1_s(:, 1), pos_a1_s(:, 2), 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'agent 1');
    plot(pos_a2_s(:, 1), pos_a2_s(:, 2), 'm-','DisplayName', 'agent 2');
    plot(pos_a3_s(:, 1), pos_a3_s(:, 2), 'g-' ,'DisplayName', 'agent 3');

    % Display the Connection Lines between Agents.
    line([pos_a0(1, :), pos_a3(1, :)], [pos_a0(2, :), pos_a3(2, :)], 'Color', 'k', 'HandleVisibility', 'off');
    line([pos_a2(1, :), pos_a1(1, :)], [pos_a2(2, :), pos_a1(2, :)], 'Color', 'k', 'HandleVisibility', 'off');
    line([pos_a0(1, :), pos_a2(1, :)], [pos_a0(2, :), pos_a2(2, :)], 'Color', 'k', 'HandleVisibility', 'off');
    line([pos_a1(1, :), pos_a3(1, :)], [pos_a1(2, :), pos_a3(2, :)], 'Color', 'k', 'HandleVisibility', 'off');




