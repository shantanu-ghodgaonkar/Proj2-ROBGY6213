%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION


for n = 2:length(sampledData)
    %% Initalize Loop load images
    
    %% Detect good points
    
    %% Initalize the tracker to the last frame.
    
    %% Find the location of the next points;
    
    %% Calculate velocity
    % Use a for loop
    
    %% Calculate Height
    
    
    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC

    %% Thereshold outputs into a range.
    % Not necessary
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 
    estimatedV(:,n) = Vel;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
