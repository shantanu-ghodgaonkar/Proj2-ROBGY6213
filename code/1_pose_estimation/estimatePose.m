function [position, orientation] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX

%% MY IMPLEMENTATION START ------------------------------------------------
% tic

% Camera intrinsic matrix
k = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1]; 

% A (8x9) matrix
A = [];

% Iterate over all the detected April Tags
for i = 1:length(data(t).id)
    % Get corners of each April Tag in the world frame
    pw = getCorner(data(t).id(i));
    % Get corners of each April Tag in the camera frame
    pc = [data(t).p1(:,i), data(t).p2(:,i), data(t).p3(:,i), data(t).p4(:,i)];
  
    % Find the A matrix 
    A_temp = [   pw(1,1), pw(2,1), 1, 0, 0, 0, (-pc(1,1)*pw(1,1)), (-pc(1,1)*pw(2,1)), (-pc(1,1));
        0, 0, 0, pw(1,1), pw(2,1), 1, (-pc(2,1)*pw(1,1)), (-pc(2,1)*pw(2,1)), (-pc(2,1));
        pw(1,2), pw(2,2), 1, 0, 0, 0, (-pc(1,2)*pw(1,2)), (-pc(1,2)*pw(2,2)), (-pc(1,2));
        0, 0, 0, pw(1,2), pw(2,2), 1, (-pc(2,2)*pw(1,2)), (-pc(2,2)*pw(2,2)), (-pc(2,2));
        pw(1,3), pw(2,3), 1, 0, 0, 0, (-pc(1,3)*pw(1,3)), (-pc(1,3)*pw(2,3)), (-pc(1,3));
        0, 0, 0, pw(1,3), pw(2,3), 1, (-pc(2,3)*pw(1,3)), (-pc(2,3)*pw(2,3)), (-pc(2,3));
        pw(1,4), pw(2,4), 1, 0, 0, 0, (-pc(1,4)*pw(1,4)), (-pc(1,4)*pw(2,4)), (-pc(1,4));
        0, 0, 0, pw(1,4), pw(2,4), 1, (-pc(2,4)*pw(1,4)), (-pc(2,4)*pw(2,4)), (-pc(2,4));];
    % Vertically stack all A matrices
    A = vertcat(A,A_temp); 
end
% Perform SVD of A to get the V matrix
[~, ~, V] = svd(A);

h = V(9,9) * [V(1:3,9), V(4:6,9), V(7:9,9)]';

% Find RT = [R1 R2 T]
RT = k\h;
% Extract R1
R1 = RT(:,1);
R1_skew = [0, -R1(3,1), R1(2,1); R1(3,1), 0, -R1(1,1); -R1(2,1), R1(1,1), 0];
% Extract R2
R2 = RT(:,2);
% Extract T
T_temp = RT(:,3);

% Compute the SVD of the newly formed orthogonal matrix
[Ur, ~, Vr] = svd([R1, R2, R1_skew*R2]);

% Compute the rotation matrix from the orthogonal matrix
R = Ur * [1, 0, 0; 0, 1, 0; 0, 0, det(Ur*Vr')] * Vr';

% Scale T
T = T_temp / norm(R1) ;

% Combine rotational and translational parts to get camera to world frame
HT = [[R, T] ; [0,0,0,1]];

% Find the Homogenous transform to go from camera frame to imu frame
Rb2c = [0.7071, -0.7071, 0; -0.7071, -0.7071, 0; 0, 0, -1]; % eul2rotm([-pi/4,pi,0])
Tb2c = [0.0283; -0.0283; 0.0300]; % eul2rotm([-pi/4,pi,0]) * [-0.04, 0.0, -0.03]';
c2b = inv([Rb2c, Tb2c; [0,0,0,1]]);

% Transform to imu frame
HT = HT\c2b;

% Set the position
position = HT(1:3, 4);

% Set the orientation
orientation = rotm2eul(HT(1:3, 1:3)) ;

% toc
%% MY IMPLEMENTATION END --------------------------------------------------
end