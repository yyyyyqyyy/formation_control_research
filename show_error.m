%% After the case simulation is completed, while preserving the workspace data, execute this function to generate plots
% A0 error in X ，Y 
    figure(2);  
    subplot(4, 2, 1);
    plot(error_x_a0_s);
    xlabel('Time');
    ylabel('A0 Pose Error in X');
    title('A0 Pose Error in X Over Time');
    grid on;

    subplot(4, 2, 2);
    plot(error_y_a0_s, 'r-');
    xlabel('Time');
    ylabel('A0 Pose Error in Y ');
    title('A0 Error in Y Over Time');
    grid on;
    %% A1 error in X ，Y 

    subplot(4, 2, 3);
    plot(error_x_a1_s);
    xlabel('Time');
    ylabel('A1 Pose Error in X');
    title('A1 Pose Error in X Over Time');
    grid on;

    subplot(4, 2, 4);   
    plot(error_y_a1_s, 'r-');
    xlabel('Time');
    ylabel('A1 Pose Error in Y');
    title('A1 Pose Error in Y Over Time');
    grid on;
    %% A2 error in X ，Y 

    subplot(4, 2, 5);
    plot(error_x_a2_s);
    xlabel('Time');
    ylabel('A2 Pose Error in X');
    title('A2 Pose Error in X Over Time');
    grid on;

    subplot(4, 2, 6);
    plot(error_y_a2_s, 'r-');
    xlabel('Time');
    ylabel('A2 Pose Error in Y');
    title('A2 Pose Error in Y Over Time');
    grid on;

    %% A3 error in X ，Y 

    subplot(4, 2, 7);
    plot(error_x_a3_s);
    xlabel('Time');
    ylabel('A3 Pose Error in X');
    title('A3 Pose Error in X Over Time');
    grid on;

    subplot(4, 2, 8);
    plot(error_y_a3_s, 'r-');
    xlabel('Time');
    ylabel('A3 Pose Error in Y');
    title('A3 Pose Error in Y Over Time');
    grid on;

%% 
%% 
% Create a 2x2 subplot area for four regions
figure(5);

% First Region: Agent 0
subplot(2, 2, 1);
scatter(error_x_a0_s, error_y_a0_s, 'Marker', '.', 'MarkerEdgeColor', 'b');
grid on;
title('Error Agent 0');

% Second Region: Agent 1
subplot(2, 2, 2);
scatter(error_x_a1_s, error_y_a1_s, 'Marker', '.', 'MarkerEdgeColor', 'b');
grid on;
title('Error Agent 1');

% Third Region: Agent 2
subplot(2, 2, 3);
scatter(error_x_a2_s, error_y_a2_s, 'Marker', '.', 'MarkerEdgeColor', 'b');
grid on;
title('Error Agent 2');

% Fourth Region: Agent 3
subplot(2, 2, 4);
scatter(error_x_a3_s, error_y_a3_s, 'Marker', '.', 'MarkerEdgeColor', 'b');
grid on;
title('Error Agent 3');

%% 

% Create a new figure (figure 7) for detailed error analysis

figure(7);

% First subplot: Agent 0
subplot(2, 2, 1); 
% Calculate the maximum and minimum values for error_x_a0_s and error_y_a0_s
max_error_x = max(error_x_a0_s);
min_error_x = min(error_x_a0_s);
max_error_y = max(error_y_a0_s);
min_error_y = min(error_y_a0_s);
line([min_error_x, max_error_x], [mean(error_y_a0_s), mean(error_y_a0_s)], 'Color', 'b', 'LineWidth', 2);
hold on;
% Draw vertical error bars in the y-direction
line([mean(error_x_a0_s), mean(error_x_a0_s)], [min_error_y, max_error_y], 'Color', 'b', 'LineWidth', 2);

plot([min_error_x, min_error_x], [mean(error_y_a0_s) - std(error_y_a0_s), mean(error_y_a0_s) + std(error_y_a0_s)], 'b', 'LineWidth', 2);
plot([max_error_x, max_error_x], [mean(error_y_a0_s) - std(error_y_a0_s), mean(error_y_a0_s) + std(error_y_a0_s)], 'b', 'LineWidth', 2);
plot([mean(error_x_a0_s) - std(error_x_a0_s), mean(error_x_a0_s) + std(error_x_a0_s)], [min_error_y, min_error_y], 'b', 'LineWidth', 2);
plot([mean(error_x_a0_s) - std(error_x_a0_s), mean(error_x_a0_s) + std(error_x_a0_s)], [max_error_y, max_error_y], 'b', 'LineWidth', 2);
scatter(mean(error_x_a0_s), mean(error_y_a0_s), 100, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b'); 
% Add labels and title
xlabel('Error in X');
ylabel('Error in Y');
title('Error of Agent 0');
grid on;

%% 

% Second subplot: Agent 1
subplot(2, 2, 2);
% Calculate the maximum and minimum values for error_x_a1_s and error_y_a1_s
max_error_x = max(error_x_a1_s);
min_error_x = min(error_x_a1_s);
max_error_y = max(error_y_a1_s);
min_error_y = min(error_y_a1_s);
% Draw horizontal error bars in the x-direction
line([min_error_x, max_error_x], [mean(error_y_a1_s), mean(error_y_a1_s)], 'Color', 'b', 'LineWidth', 2);
hold on;
% Draw vertical error bars in the y-direction
line([mean(error_x_a1_s), mean(error_x_a1_s)], [min_error_y, max_error_y], 'Color', 'b', 'LineWidth', 2);
line_length = 0.2;
plot([min_error_x, min_error_x], [mean(error_y_a1_s) - std(error_y_a1_s), mean(error_y_a1_s) + std(error_y_a1_s)], 'b', 'LineWidth', 2);
plot([max_error_x, max_error_x], [mean(error_y_a1_s) - std(error_y_a1_s), mean(error_y_a1_s) + std(error_y_a1_s)], 'b', 'LineWidth', 2);
plot([mean(error_x_a1_s) - std(error_x_a1_s), mean(error_x_a1_s) + std(error_x_a1_s)], [min_error_y, min_error_y], 'b', 'LineWidth', 2);
plot([mean(error_x_a1_s) - std(error_x_a1_s), mean(error_x_a1_s) + std(error_x_a1_s)], [max_error_y, max_error_y], 'b', 'LineWidth', 2);
scatter(mean(error_x_a1_s), mean(error_y_a1_s), 100, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b'); 
% Add labels and title
xlabel('Error in X');
ylabel('Error in Y');
title 'Error of Agent 1';
grid on;


%% 

% Third subplot: Agent 2
subplot(2, 2, 3); 
% Calculate the maximum and minimum values for error_x_a2_s and error_y_a2_s
max_error_x = max(error_x_a2_s);
min_error_x = min(error_x_a2_s);
max_error_y = max(error_y_a2_s);
min_error_y = min(error_y_a2_s);
% Draw horizontal error bars in the x-direction
line([min_error_x, max_error_x], [mean(error_y_a2_s), mean(error_y_a2_s)], 'Color', 'b', 'LineWidth', 2);
hold on;
% Draw vertical error bars in the y-direction
line([mean(error_x_a2_s), mean(error_x_a2_s)], [min_error_y, max_error_y], 'Color', 'b', 'LineWidth', 2);
line_length = 0.2;
plot([min_error_x, min_error_x], [mean(error_y_a2_s) - std(error_y_a2_s), mean(error_y_a2_s) + std(error_y_a2_s)], 'b', 'LineWidth', 2);
plot([max_error_x, max_error_x], [mean(error_y_a2_s) - std(error_y_a2_s), mean(error_y_a2_s) + std(error_y_a2_s)], 'b', 'LineWidth', 2);
plot([mean(error_x_a2_s) - std(error_x_a2_s), mean(error_x_a2_s) + std(error_x_a2_s)], [min_error_y, min_error_y], 'b', 'LineWidth', 2);
plot([mean(error_x_a2_s) - std(error_x_a2_s), mean(error_x_a2_s) + std(error_x_a2_s)], [max_error_y, max_error_y], 'b', 'LineWidth', 2);
scatter(mean(error_x_a2_s), mean(error_y_a2_s), 100, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b'); 
% Add labels and title
xlabel('Error in X');
ylabel('Error in Y');
title('Error of Agent 2');
grid on;


%% 

% Fourth subplot: Agent 3
subplot(2, 2, 4); 
% Calculate the maximum and minimum values for error_x_a3_s and error_y_a3_s
max_error_x = max(error_x_a3_s);
min_error_x = min(error_x_a3_s);
max_error_y = max(error_y_a3_s);
min_error_y = min(error_y_a3_s);
% Draw horizontal error bars in the x-direction
line([min_error_x, max_error_x], [mean(error_y_a3_s), mean(error_y_a3_s)], 'Color', 'b', 'LineWidth', 2);
hold on;
% Draw vertical error bars in the y-direction
line([mean(error_x_a3_s), mean(error_x_a3_s)], [min_error_y, max_error_y], 'Color', 'b', 'LineWidth', 2);
line_length = 0.2;
plot([min_error_x, min_error_x], [mean(error_y_a3_s) - std(error_y_a3_s), mean(error_y_a3_s) + std(error_y_a3_s)], 'b', 'LineWidth', 2);
plot([max_error_x, max_error_x], [mean(error_y_a3_s) - std(error_y_a3_s), mean(error_y_a3_s) + std(error_y_a3_s)], 'b', 'LineWidth', 2);
plot([mean(error_x_a3_s) - std(error_x_a3_s), mean(error_x_a3_s) + std(error_x_a3_s)], [min_error_y, min_error_y], 'b', 'LineWidth', 2);
plot([mean(error_x_a3_s) - std(error_x_a3_s), mean(error_x_a3_s) + std(error_x_a3_s)], [max_error_y, max_error_y], 'b', 'LineWidth', 2);
scatter(mean(error_x_a3_s), mean(error_y_a3_s), 100, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b'); 
% Add labels and title
xlabel('Error in X');
ylabel('Error in Y');
title('Error of Agent 3');
grid on;
