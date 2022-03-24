%RUNMODEL - Main code to run robot model for ENG5009 class
%
%   NOTE: Students should only change the code between *---* markers
%
% Syntax: RunModel
%
% Inputs: none
% Outputs: none
%
% Other m-files required:
%   Sensor.m
%   WallGeneration.m
%   DynamicalModel.m
%   DrawRobot.m
%
% Author: Dr. Kevin Worrall
% Last revision: 06-01-2021

%% Preamble
close all;
clear all;
clc;
addpath('..\controllers');
addpath('..\source');
%% Simulation setup
% Time
simulationTime_total = 400;           % in seconds *------* YOU CAN CHANGE THIS
stepSize_time = 0.05;               % in seconds

% Initial states and controls
voltage_left  = 6;                  % in volts *------* YOU CAN CHANGE THIS
voltage_right = 6;                  % in volts *------* YOU CAN CHANGE THIS
state_initial = zeros(1,24);        % VARIABLES OF IMPORTANCE:
state_initial(19)= -1;

% state_initial(13): forward velocity,    v, m/s
% state_initial(18): rotational velocity, r, rad/s
% state_initial(19): current x-position,  x, m
% state_initial(20): current y-position,  y, m
% state_initial(24): heading angle,       psi, rad

% Environment
canvasSize_horizontal = 10;
canvasSize_vertical   = 10;
stepSize_canvas       = 0.01;

% *------------------------------------------*
%  YOU CAN ADD MORE SETUP HERE
%  (e.g. setup of controller or checkpoints)
% *------------------------------------------*
k = 1;
checkPointList = [ 2 3; 1 4; 3 -4; -1 -2; 3 0; 1.5 2];
checkPointCounter = 1;
lastCheckPoint = 0;
tolerance = 0.05; 
currentHeadingAngle = 0;
%weights = [-0.5 1 1 -0.5]; % weights = [ w1 w2 w3 w4];
%thresholds = [0.4 0.4]; % thresholds = [mr ml];
%% Create Environment
obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

% Generate walls
% --> the variable "obstacleMatrix" is updated for each added wall
%[wall_1, obstacleMatrix] = WallGeneration( -1,  1, 1.2, 1.2, 'h', obstacleMatrix);
%[wall_2, obstacleMatrix] = WallGeneration( -3, -3,  -2,   2, 'v', obstacleMatrix);

% *---------------------------*
%  YOU CAN ADD MORE WALLS HERE
% *---------------------------*
[wall_1, obstacleMatrix] = WallGeneration( -4, 0.5, 1, 1, 'h', obstacleMatrix);
[wall_2, obstacleMatrix] = WallGeneration( 2, 2, -1, 1, 'v', obstacleMatrix);
[wall_3, obstacleMatrix] = WallGeneration( -2.5, -2.5, 2.5, 5, 'v', obstacleMatrix);
%% Main simulation
% Initialize simulation
timeSteps_total = simulationTime_total/stepSize_time;
state = state_initial;
time = 0;
%avoidance_control = readfis('avoidanceControl');
% voltage.left = voltage_left;
% voltage.right = voltage_right;

% Run simulation
for timeStep = 1:timeSteps_total
    if checkPointCounter ~= lastCheckPoint
        t_start = tic;
        lastCheckPoint = checkPointCounter;
    end
    % Assign voltage applied to wheels
    voltages = [voltage_left; voltage_left; voltage_right; voltage_right];
    sensorOut = Sensor(state(timeStep,19), state(timeStep,20), ...
        state(timeStep,24), obstacleMatrix);

    % *-------------------------------------*
    %  YOU CAN ADD/CHANGE YOUR CONTROLS HERE
    % *-------------------------------------*
    currentLocation = state(timeStep, 19:20);
    currentHeadingAngle = state(timeStep,24);
    if checkPointCounter == length(checkPointList)+1
        fprintf('You have succesffully navigated the path \n');
        pause(1)
        fprintf('Reward yourself with a pint or something \n');

        fprintf('Elon Musk has nothing on you \n')
        break
    end
    checkpoint = checkPointList(checkPointCounter,:);
    [booleanAtCheckPoint, newHeadingAngle] = ComputeHeadingAngle(currentLocation, checkpoint, tolerance);
    
    [error] = errorCalculator(newHeadingAngle, currentHeadingAngle);
    [checkPointCounter,voltage] = fuzzyTaskOneController(booleanAtCheckPoint, checkPointCounter, error, sensorOut);
    if booleanAtCheckPoint == 1
        t_end = toc(t_start);
        times(checkPointCounter-1) = t_end
    end
    voltage_right = voltage(:,1);
    voltage_left = voltage(:,2);

    voltages = [voltage_left; voltage_left; voltage_right; voltage_right];
    % Run model *** DO NOT CHANGE
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);
   
    % Euler intergration *** DO NOT CHANGE
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time);
    time(timeStep + 1)    = timeStep * stepSize_time;
    
   % currentHeadingAngle = newHeadingAngle;

    % Plot robot on canvas  *------* YOU CAN ADD STUFF HERE
    checkPointListPlot = [ 3 2; 4 1; -4 3; -2 -1; 0 3; 2 1.5];
    figure(1); clf; hold on; grid on; axis([-5,5,-5,5]);
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-');
    plot(wall_3(:,1), wall_3(:,2),'k-');
    plot(checkPointListPlot(:,1), checkPointListPlot(:,2), 'o');
    xlabel('y, m'); ylabel('x, m');
    % a wee counter for me
    k = k+1;
end
    

%% Plot results
% *----------------------------------*
%  YOU CAN ADD OR CHANGE FIGURES HERE
%  don't forget to add axis labels!
% *----------------------------------*

figure(2); hold on; grid on;
plot(state(:,20), state(:,19));
plot(wall_1(:,1), wall_1(:,2),'k-');
plot(wall_2(:,1), wall_2(:,2),'k-');
plot(wall_3(:,1), wall_3(:,2),'k-');
plot(checkPointListPlot(:,1), checkPointListPlot(:,2), 'o');
title('Plot of X-Y Position')

xlabel('Y Position (m)')
ylabel(' X Position (m)')

figure(3); hold on; grid on;
plot(time, state(:,19));
title('x Distance Travelled')
ylabel('x Distance (m)')
xlabel('time (s)')
figure(4); hold on; grid on;
plot(time, state(:,24));
title('Heading Angle During Clockwise Rotation')
ylabel('Heading angle (rad)')
xlabel('time (s)')
