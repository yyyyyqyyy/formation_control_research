% getLidarScan : Generate simulated Lidar Scan data and create a Lidar scan object.

function scan = getLidarScan(AgentCurrentPose, map)

%         input:
%                 AgentCurrentPose = [x_a, y_a, theta_a]: position of the agent in a 3x1 list
%                 map: grid map
%         return:
%                  scan: create a Lidar scan object for MCL


    maxRange = 30; % The maximum measurement distance is 30 meters.
    angleIncrement = pi/360; % Angular resolution: The angle increment is 1 degrees, which is equivalent to 1 degrees per laser beam.
    distance_resolution = 0.1; % Distance resolution

    % Generate the angle vector 'angles' for the lidar scan from -pi to pi.
    angles = -pi:angleIncrement:pi;

    % Initialize the measurement data 'ranges' for the lidar.
    ranges = maxRange * ones(1, numel(angles));

    


    % Iterate through all the specified angles to calculate lidar measurement data.
    for i = 1:numel(angles)
        % Get the current angle for the laser beam.
        angle = angles(i);

        % Extract the agent's current position and orientation.
        x0 = AgentCurrentPose(1);
        y0 = AgentCurrentPose(2);
        theta0 = AgentCurrentPose(3);

        % Initialize x and y at the agent's current position.
        x = x0;
        y = y0;
        
        % Simulate the laser beam's interaction with the map.
        % Iterate while the map occupancy at the current position is less than 0.5
        % and the distance from the starting point does not exceed 'maxRange'.
        % and the distance from the starting point does not exceed 'maxRange'.
        while map.getOccupancy([x, y]) < 0.5 && norm([x-x0, y-y0]) <= maxRange
            x = x + cos(theta0 + angle) * distance_resolution; 
            y = y + sin(theta0 + angle) * distance_resolution;
        end
        ranges(i) = norm([x-x0, y-y0]);
    end

    % create a Lidar scan object ：The lidar scan is a laser scan for a 2-D plane with distances () measured from the sensor 
    % to obstacles in the environment at specific angles ().
    %  https://de.mathworks.com/help/nav/ref/lidarscan.html?searchHighlight=lidarScan%28ranges%2C%20angles%29&s_tid=srchtitle_support_results_1_lidarScan%2528ranges%252C%20angles%2529
    scan = lidarScan(ranges, angles);
end