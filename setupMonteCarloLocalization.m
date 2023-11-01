% setupMonteCarloLocalization : Creates a Monte Carlo localization (MCL) object, and set the parameters for Monte Carlo localization

%     A2 ———————————————————— A0
%     |                       |
%     |           VC          |
%     |                       |
%     A1 ———————————————————— A3   
%   
%  

function [mcl] = setupMonteCarloLocalization(map, pos_a)

%         input:
%                  map：grid map
%                  pos_a: Initial pose of Agent, used to start localization

%         return:
%                  mcl: Creates a Monte Carlo localization (MCL) object



    % Create a Monte Carlo Localization (MCL) object from Official MATLAB library. 
    % Estimates the pose and covariance of a vehicle using the MCL algorithm.
    % https://de.mathworks.com/help/nav/ref/montecarlolocalization-system-object.html
    mcl = monteCarloLocalization;

    % mcl : Estimates the pose and covariance of a Agent using the MCL algorithm.
    %         input:
    %                 odometryPose_a = [x_a, y_a, theta]: pose of the agent 0. Using control algorithms to calculate the position as odometer data.
    %                 map: grid map
    %         return:
    %                  isUpdated_a: Flag for pose update, returned as a logical.
    %                  estimatedPose_a :Current pose estimate
    %                  covariance_a: Covariance estimate for current pose, returned as a matrix. This matrix gives an estimate of the uncertainty of the current pose.

    
    % Set MCL parameters
    % Use lidarScan object as scan input
    mcl.UseLidarScan = true;

    % Flag to start global localization
    mcl.GlobalLocalization = false;

    % Set MCL Particle Limits : Minimum and maximum number of particles, specified as a two-element vector, .[min max]
    mcl.ParticleLimits = [500 5000];

    % Set Initial pose of Agent
    mcl.InitialPose = pos_a;

    % Set Covariance of initial pose:  Covariance of the Gaussian distribution for the initial pose, specified as a diagonal matrix. 
    % Three-element vector and scalar inputs are converted to a diagonal matrix. This matrix gives an estimate of the uncertainty of the .InitialPose
    mcl.InitialCovariance = eye(3) * 0.05;
    
    % Create a Likelihood Field Sensor Model
    sm = likelihoodFieldSensorModel;
    sm.Map = map;
    sm.SensorLimits =  [0.45 30];
    
    % Set the sensor model for MCL
    mcl.SensorModel = sm;
    
    % Set Minimum change in states required to trigger update
    % Minimum change in states required to trigger update, specified as a three-element vector. 
    % The localization updates the particles if the minimum change in any of the states is met. The pose estimate updates only if the particle filter is updated.[x y theta]
    mcl.UpdateThresholds = [0.01, 0.01, 0.1];

    % Number of filter updates between resampling of particles: Number of filter updates between resampling of particles, specified as a positive integer.
    mcl.ResamplingInterval = 20;
end
