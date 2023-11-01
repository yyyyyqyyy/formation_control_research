clear all;
load('mapmatrix.mat');
map = binaryOccupancyMap(mapmatrix,1);
% map = binaryOccupancyMap(17,27,1);


maxRange = 30; % The maximum measurement distance is 30 meters.
angleIncrement = pi/360; % The angle increment is 1 degrees, which is equivalent to 1 degrees per laser beam.

robotCurrentPose = [5,10,0];
visualizeLidarScan(robotCurrentPose, map, maxRange, angleIncrement)