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

k = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];
% R = eye(3,3);
% R_temp = eye(3,3);
% T = eye(3,1);
% T_temp = eye(3,1);
for i = 1:length(data(t).id)
    pw = getCorner(data(t).id(i));
    pc = [data(t).p0(:,i), data(t).p1(:,i), data(t).p2(:,i), data(t).p3(:,i), data(t).p4(:,i)];
    for j = 2:length(pw)
        if (j == 2)
            A = [pw(1,j), pw(2,j), 1, 0, 0, 0, (-pc(1,j)*pw(1,j)), (-pc(1,j)*pw(2,j)), (-pc(1,j));
                0, 0, 0, pw(1,j), pw(2,j), 1, (-pc(2,j)*pw(1,j)), (-pc(2,j)*pw(2,j)), (-pc(2,j))];
        else
            A = vertcat(A, [pw(1,j), pw(2,j), 1, 0, 0, 0, (-pc(1,j)*pw(1,j)), (-pc(1,j)*pw(2,j)), (-pc(1,j));
                0, 0, 0, pw(1,j), pw(2,j), 1, (-pc(2,j)*pw(1,j)), (-pc(2,j)*pw(2,j)), (-pc(2,j))]);
        end
    end
    [~, ~, V] = svd(A);
    h = sign(V(9,9))*[V(1:3,9), V(4:6,9), V(7:9,9)];
    % RT = k \ h;
    % R1 = RT(:,1);
    % R1_hat = [0, -R1(3,1), R1(2,1); R1(3,1), 0, -R1(1,1); -R1(2,1), R1(1,1), 0];
    % R2 = RT(:,2);
    % R2_hat = [0, -R2(3,1), R2(2,1); R2(3,1), 0, -R2(1,1); -R2(2,1), R2(1,1), 0];
    % T = RT(:,3);
    % T_hat = [0, -T(3,1), T(2,1); T(3,1), 0, -T(1,1); -T(2,1), T(1,1), 0];
    % [Ur, ~, Vr] = svd(cross((R1_hat * R2_hat * R1_hat), R2_hat));
    % if i == 1
    %     R = Ur * [1, 0, 0; 0, 1, 0; 0, 0, det(Ur*Vr')] * Vr';
    %     T_temp = T_hat / norm(R1_hat);
    %     T = [T_temp(3,2); -T_temp(3,1); T_temp(2,1)];
    % else
    %     R_temp = Ur * [1, 0, 0; 0, 1, 0; 0, 0, det(Ur*Vr')] * Vr';
    %     R = leastSquaresAvg(R, R_temp);
    %     T_temp = T_hat / norm(R1_hat);
    %     T_temp = [T_temp(3,2); -T_temp(3,1); T_temp(2,1)];
    %     T = leastSquaresAvg(T, T_temp);
    % end

end

RT = k \ h;
R1 = RT(:,1);
R1_hat = [0, -R1(3,1), R1(2,1); R1(3,1), 0, -R1(1,1); -R1(2,1), R1(1,1), 0];
R2 = RT(:,2);
R2_hat = [0, -R2(3,1), R2(2,1); R2(3,1), 0, -R2(1,1); -R2(2,1), R2(1,1), 0];
T = RT(:,3);
T_hat = [0, -T(3,1), T(2,1); T(3,1), 0, -T(1,1); -T(2,1), T(1,1), 0];
[Ur, ~, Vr] = svd(cross((R1_hat * R2_hat * R1_hat), R2_hat));

R = Ur * [1, 0, 0; 0, 1, 0; 0, 0, det(Ur*Vr')] * Vr';
T_temp = T_hat / norm(R1_hat);
T = [T_temp(3,2); -T_temp(3,1); T_temp(2,1)];
% disp("DEBUG POINT")
% [Ur, ~, Vr] = svd(cross((R1 * R2 * R1), R2));
% R = Ur * [1, 0, 0; 0, 1, 0; 0, 0, det(Ur*Vr')] * Vr';
position = T;
orientation = rotm2eul(R);

% toc
%% MY IMPLEMENTATION END --------------------------------------------------
end