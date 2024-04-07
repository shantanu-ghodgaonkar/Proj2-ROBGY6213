tic%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);
%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
%% MY IMPLEMENTATION START ------------------------------------------------
k = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210;0, 0, 1];
b2c = [0.7071, -0.7071, 0, 0.0283; -0.7071, -0.7071, 0, -0.0283; 0, 0, -1, 0.03; 0, 0, 0, 1];
R_c2b=rotz(-pi/4)*rotx(pi); % Camera to Bot Rotation
Skew_t_c2b=[0,0.03,0;-0.03,0,0.04;0,-0.04,0];
t = sgolayfilt([sampledData.t],1,101);
strongestPointsNum = 100;
ransacFlag = 0;
for n = 2:length(sampledData)
    %% Initalize Loop load images
    prevImg = sampledData(n-1).img;
    currImg = sampledData(n).img;
    %% Detect good points
    prevPts = detectHarrisFeatures(prevImg).selectStrongest(strongestPointsNum).Location; % Calibrate to reduce this number
    %% Initalize the tracker to the last frame.
    pointTracker = vision.PointTracker('MaxBidirectionalError',1);
    initialize(pointTracker, prevPts, prevImg);
    %% Find the location of the next points;
    [currPts, ~, ~] = pointTracker(currImg);
    %% normalise the points 
    prevPts = k \ [prevPts, ones(strongestPointsNum,1)]';
    prevPts = [prevPts(1,:); prevPts(2,:)];
    currPts = k \ [currPts, ones(strongestPointsNum,1)]';
    currPts = [currPts(1,:); currPts(2,:)];
    %% Calculate velocity
    
    
    [position, orientation, R_c2w] = estimatePose(sampledData, n);
    R_b2w=R_c2b'*R_c2w(1:3,1:3);
    P = -1 * (b2c(1:3, 1:3) * position);
    Z = P(3);
    V_OMG = [];
    if ransacFlag == 1
        disp("DEBUG POINT");
    else
        H = [];
        p_dot = [];
        dt = t(n) - t(n-1);
        for i = 1:length(currPts)
            x_p_i = prevPts(1,i);
            y_p_i = prevPts(2,i);
            p_dot_i = [(currPts(1,i) - prevPts(1,i)); (currPts(2,i) - prevPts(2,i))]./dt;
            A_p = [-1/Z, 0, x_p_i/Z; 0, -1/Z, y_p_i/Z];
            B_p = [(x_p_i*y_p_i), -(1 + (x_p_i^2)), y_p_i; (1 + (y_p_i^2)), (-x_p_i*y_p_i), -x_p_i];
            H = vertcat(H, [((1/Z) .* A_p), B_p]);
            p_dot = vertcat(p_dot, p_dot_i);
        end
        V_OMG = [R_b2w', zeros(3); zeros(3) R_b2w'] * ([R_c2b -R_c2b*Skew_t_c2b;zeros(3,3) R_c2b] * (pinv(H) * p_dot));
    end

    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) = V_OMG; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z

end
estimatedV=sgolayfilt(double(estimatedV'),1,25)';
%% MY IMPLEMENTATION END --------------------------------------------------
plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
toc