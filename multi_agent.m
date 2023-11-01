



%% 



% build map
clear all;
load('mapmatrix.mat');
map = binaryOccupancyMap(mapmatrix,1);
% show(map)
%% 
robot = differentialDriveKinematics("WheelRadius",0.1525,"TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");
robot2 = differentialDriveKinematics("WheelRadius",0.1525,"TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");

%% inflated map
mapInflated = copy(map);
inflate(mapInflated, robot.TrackWidth/2);
show(mapInflated)

%% Astar path planing
planner = plannerAStarGrid(map);
startLocation = [6.0 10.0];
endLocation = [13.0 10.0];
path = plan(planner,startLocation,endLocation);


% %% Astar path planing
% planner2 = plannerAStarGrid(map);
% startLocation2 = [6.0 14.0];
% endLocation2 = [13.0 14.0];
% path2 = plan(planner2,startLocation2,endLocation2);

%% define controller
% controller = controllerPurePursuit;
controller = controllerPurePursuit('DesiredLinearVelocity', 0.5);
release(controller);
controller.Waypoints = path;
robotInitialLocation = path(1,:);  %start and end point
robotGoal = path(end,:);
initialOrientation = 0;  
robotCurrentPose = [robotInitialLocation initialOrientation]';  %initial position
distanceToGoal = norm(robotInitialLocation - robotGoal); %Calculate the distance to the target position
goalRadius = 0.1;
% % % % % 
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
reset(vizRate);
% Initialize the figure
% figure;
frameSize = robot.TrackWidth/0.8;

% 
% % define controller2
% controller = controllerPurePursuit;
% controller2 = controllerPurePursuit('DesiredLinearVelocity', 0.5);
% release(controller);
% controller2.Waypoints = path2;
% robotInitialLocation2 = path2(1,:);  %start and end point
% robotGoal2 = path2(end,:);
% initialOrientation2 = 0;  
% robotCurrentPose2 = [robotInitialLocation2 initialOrientation2]';  %initial position
% distanceToGoal2 = norm(robotInitialLocation2 - robotGoal2); %Calculate the distance to the target position
% goalRadius = 0.1;
% % % % % % 
% sampleTime = 0.1;
% vizRate = rateControl(1/sampleTime);
% reset(vizRate);
% % Initialize the figure
% % figure;
% 8;



%% 
maxRange = 30; % 最大测量距离为 25 米
angleIncrement = pi/360; % 角度增量为 0.5°，即 0.5° 一个激光束
%% 
mcl = monteCarloLocalization;
mcl.UseLidarScan = true;
% mcl.GlobalLocalization = true;
mcl.GlobalLocalization = false;
mcl.ParticleLimits = [5000 50000];
mcl.InitialPose = robotCurrentPose;
mcl.InitialCovariance = eye(3)*0.5;
sm = likelihoodFieldSensorModel;
sm.Map = map;
sm.SensorLimits = [0.45 30];
mcl.SensorModel = sm;
mcl.UpdateThresholds = [0.01,0.01,0.01];
mcl.ResamplingInterval = 10;
mcl.MotionModel.Noise = [0.25 0.25 0.4 0.4];




%% 
angle = pi/2;
distance = 2;

x1 = 6.0;
y1 = 10.0;

delta_x = distance * cos(angle);
delta_y = distance * sin(angle);

x2 = x1 + delta_x;
y2 = y1 + delta_y;

robotCurrentPose2 = [x2, y2, 0]';

k1 = 0.7;
k2 = 0.7;
d = 0.1;
mcl2 = monteCarloLocalization;
mcl2.UseLidarScan = true;
% mcl.GlobalLocalization = true;
mcl2.GlobalLocalization = false;
mcl2.ParticleLimits = [5000 50000];
mcl2.InitialPose = robotCurrentPose2;
mcl2.InitialCovariance = eye(3)*0.5;
sm2 = likelihoodFieldSensorModel;
sm2.Map = map;
sm2.SensorLimits = [0.45 30];
mcl2.SensorModel = sm2;
mcl2.UpdateThresholds = [0.01,0.01,0.01];
mcl2.ResamplingInterval = 10;
% mcl2.MotionModel.Noise = [0.25 0.25 0.4 0.4];

%% 
estimatedPoses = []; % 用于存储估计的机器人位姿
estimatedPoses2 = []; % 用于存储估计的机器人位姿
truePositions = [];% 用于存储机器人的真实位置
poseError_XYs = []; 
poseError_Theta_s= [];
estimatedPose = robotCurrentPose; %初始化位置
estimatedPose2 = robotCurrentPose2;


while ((distanceToGoal > goalRadius))
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);

    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);

    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel * sampleTime; 

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
%% 

    % Update estimatedPose using MCL
    scan = getLidarScan(robotCurrentPose, map, maxRange, angleIncrement);
    scan2 = getLidarScan(robotCurrentPose2, map, maxRange, angleIncrement);
%     odometryPose = [10 10 0];
%     odometryPose = estimatedPose;
    odometryPose = robotCurrentPose(1:3)';
    odometryPose2 =  robotCurrentPose2(1:3)';
    [isUpdated, estimatedPose, covariance1] = mcl(odometryPose, scan);
    [isUpdated2, estimatedPose2, covariance2] = mcl2(odometryPose2, scan2);

    xl = estimatedPose(:,1);
    yl = estimatedPose(:,2);
    thl = estimatedPose(:,3);

    xf = estimatedPose2(:,1);
    yf = estimatedPose2(:,2);
    thf = estimatedPose2(:,3);
%     xf = robotCurrentPose2(1);
%     yf = robotCurrentPose2(2);
%     thf = robotCurrentPose2(3);

    ltx = (xf+d*cos(thf) - xl)*cos(thl) + (yf+d*sin(thf) - yl)*sin(thl);
    lty = (yf+d*sin(thf) - yl)*cos(thl) + (xf+d*cos(thf) - xl)*sin(thl);

    lzx = distance* cos(angle);
    lzy = distance* sin(angle);

    epsilon = thf -thl;

    ex = lzx - ltx;
    ey = lzy - lty;

    wl = vel(3);
    vl = sqrt(vel(1).^2 + vel(2).^2);

    vf = (k1 * ex + vl- lty * wl ) * cos(epsilon) + (k2 * ey + ltx * wl) * sin(epsilon);
    wf =((-k1 * ex - vl+ lty * wl ) * sin(epsilon)-(-k2 * ey - ltx * wl) * cos(epsilon))/d; 

    new_thf = thf + wf * sampleTime;
    new_xf = xf + sampleTime  * vf * cos(new_thf);
    new_yf = yf + sampleTime  * vf * sin(new_thf);

    robotCurrentPose2 = [new_xf, new_yf, new_thf]'; 


    
    
    %% 
    % Update the plot
    hold off
    show(map);
    hold all
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:, 1), path(:, 2), "k--d")
    
    % Plot the path of the robot as a set of transforms
    plotTrVec1 = [robotCurrentPose(1:2); 0];
    plotRot1 = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec1', plotRot1, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);

%     plot(path2(:, 1), path2(:, 2), "k--d")
    plotTrVec2 = [robotCurrentPose2(1:2); 0];
    plotRot2 = axang2quat([0 0 1 robotCurrentPose2(3)]);
    plotTransforms(plotTrVec2', plotRot2, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View", "2D", "FrameSize", frameSize);



    light;
    xlim([0 27])
    ylim([0 17])


    

    % Store estimatedPose for later plotting
    estimatedPoses = [estimatedPoses; estimatedPose(:,1), estimatedPose(:,2), estimatedPose(:,3)];
    estimatedPoses2 = [estimatedPoses2; estimatedPose2(:,1), estimatedPose2(:,2), estimatedPose2(:,3)];
    truePositions=  [truePositions; robotCurrentPose'];

%     truePosition = robotCurrentPose(1:2)';
%     estimatedPosition = estimatedPose(1:2);
    % 计算位置误差
    poseError_XY = norm(robotCurrentPose(1:2,:)'- estimatedPose(:,1:2));
    poseError_Theta = (robotCurrentPose(3,:)'- estimatedPose(:,3));

    
    % 将误差存储到数组中
    poseError_XYs = [poseError_XYs; poseError_XY];
    poseError_Theta_s = [poseError_Theta_s; poseError_Theta];

%     ex_s = [ex_s; ex];
%     ey_s = [ey_s; ey];
    
    % Plot estimatedPose trajectory
    plot(estimatedPoses(:, 1), estimatedPoses(:, 2), 'r-');
    plot(estimatedPoses2(:, 1), estimatedPoses2(:, 2), 'b-');
    waitfor(vizRate);
end
