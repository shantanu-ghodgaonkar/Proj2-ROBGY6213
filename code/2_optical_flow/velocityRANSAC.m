function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow (p_dot)
    % optPos = Position of the features in the camera frame (prevPts)
    % Z = Height of the drone 
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter    
    % M = 3
    % p_success = 0.9 to 0.95
    % beta is around 0.1
    
    %% Set the prerequisites for RANSAC
    % Probabilty that the point is an inlier
    p_success = 0.925;
    % Number of points needed
    M = 3;
    % Number of loop iterations calculation
    k = log(1-p_success) / log(1-(e^M));
    % Greatest number of inliers
    maxInlier = 0;
    %Condition for inlier
    beta = 0.1;

    for i = 1:k 
        % Find M number of random points in the given set
        randomPose = randperm(length(optPos), 3);
        p1 = optPos(:, randomPose(1,1));

        p2 = optPos(:, randomPose(1,2));

        p3 = optPos(:, randomPose(1,3));
        
        % H = [H1; H2; H3]
        H = [ [[-1/Z, 0, p1(1,1)/Z; 0, -1/Z, p1(2,1)/Z], [p1(1,1)*p1(2,1) ... 
        -(1 + (p1(1,1)^2)), p1(2,1); (1 + (p1(2,1)^2)), -p1(1,1)*p1(2,1), -p1(1,1)]];
        [[-1/Z, 0, p2(1,1)/Z; 0, -1/Z, p2(2,1)/Z], [p2(1,1)*p2(2,1) ... 
        -(1 + (p2(1,1)^2)), p2(2,1); (1 + (p2(2,1)^2)), -p2(1,1)*p2(2,1), -p2(1,1)]];
        [[-1/Z, 0, p3(1,1)/Z; 0, -1/Z, p3(2,1)/Z], [p3(1,1)*p3(2,1) ... 
        -(1 + (p3(1,1)^2)), p3(2,1); (1 + (p3(2,1)^2)), -p3(1,1)*p3(2,1), -p3(1,1)]]];

        p_dot = [optV(2*randomPose(1,1) - 1); optV(2*randomPose(1,1)); optV(2*randomPose(1,2) - 1); 
                optV(2*randomPose(1,2)); optV(2*randomPose(1,3) - 1); optV(2*randomPose(1,3))];

        % Compute the velocites from randomly chosen points
        V_OMG = pinv(H)*p_dot;

        % Set the counter for number of inliers for this iteration to 0
        iterInlier = 0;

        % Iterate over all points in this set to count the number of inliers
        for j = 1:length(optPos)
            H_j = [[-1/Z, 0, optPos(1,j)/Z; 0, -1/Z, optPos(2,j)/Z], [optPos(1,j)*optPos(2,j) ... 
        -(1 + (optPos(1,j)^2)), optPos(2,j); (1 + (optPos(2,j)^2)), -optPos(1,j)*optPos(2,j), -optPos(1,j)]];
            p_dot_j = [optV(2*j - 1); optV(2*j)];
            diff = norm(H_j*V_OMG - p_dot_j);
            % Condition for inlier
            if diff < beta
                iterInlier = iterInlier + 1;
            end
        end
        
        % Check if the number of inlier counted is greater than the
        % greatest one counted yet. 
        if iterInlier >= maxInlier
            maxInlier = iterInlier;
            Vel = V_OMG;
        end
    end 

    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
end