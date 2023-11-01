function visualizeLidarScan(robotCurrentPose, map, maxRange, angleIncrement)
    % Generate the angle vector 'angles' for the Lidar scan from -pi to pi
    angles = -pi:angleIncrement:pi;
    
    % Initialize the Lidar's measurement data 'ranges'
    ranges = maxRange * ones(1, numel(angles));
    
    % Calculate the Lidar measurement data
    for i = 1:numel(angles)
        % Get the measurement distance of the Lidar beam at the current angle on the map.
        % Here, a simple ray tracing method is used to obtain the measurement distance. More complex methods may be used in practical applications.
        % Assume the robot's current position is (x0, y0), with an angle of theta0, and the Lidar beam's angle is 'angle'.
        angle = angles(i);
        x0 = robotCurrentPose(1);
        y0 = robotCurrentPose(2);
        theta0 = robotCurrentPose(3);
        x = x0;
        y = y0;
        while map.getOccupancy([x, y]) < 0.5 && norm([x-x0, y-y0]) <= maxRange
            x = x + cos(theta0 + angle) * 0.1; % Step size is 0.1 meters, which can be adjusted according to the actual situation.
            y = y + sin(theta0 + angle) * 0.1;
        end
        ranges(i) = norm([x-x0, y-y0]);
    end

    % Create a Lidar scan object 'scan'
    scan = lidarScan(ranges, angles);

    % Offset the Scan obtained from Lidar to align with the grid map and visualize the scan
    scan_In_map = transformScan(scan, [x0 y0 0]);

    figure;
    hold on;
    % Display the grid map
    show(map);
    % Display the robot's current position and orientation
    plot(robotCurrentPose(1), robotCurrentPose(2), 'bo', 'MarkerSize', 10); % Represent the robot's current position with a blue dot
    quiver(robotCurrentPose(1), robotCurrentPose(2), cos(deg2rad(robotCurrentPose(3))), sin(deg2rad(robotCurrentPose(3))), 'b', 'LineWidth', 1.5); % Draw an arrow to indicate the robot's orientation
    % Display the Lidar scan information
    plot(scan_In_map);

    hold off;
    axis equal;
    grid on;
    title('LiDAR scan map visualization');
    xlabel('X (m)');
    ylabel('Y (m)');
    legend('Robot', 'Robot Orientation', 'Scan Point', 'Lidar Scan');
end


