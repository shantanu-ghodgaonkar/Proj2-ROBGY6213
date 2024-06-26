function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method

%% MY IMPLEMENTATION START ------------------------------------------------
%tic
col = floor((id / 12)) + 1; % Calculate the column number of the tag
row = mod(id, 12) + 1; % Calculate the row number of the tag
fullTagSize = 0.152; % Set the tag size
p4X = 2 * ((row - 1) * 0.152); % Calculate the X coordinate of p4

% Calculate the Y coordinate of p4 based on the column number
% accounting for the uneven column spacing
if(col <= 3)
    p4Y = 2 * ((col - 1) * 0.152);
elseif ((col > 3) && (col <= 6))
    p4Y = ((2 * (col - 1)) * 0.152) + 0.026;
elseif (col > 6)
    p4Y = ((2 * (col - 1)) * 0.152) + 0.052;
end

res = [... [(p4X + halfTagSize); (p4Y + halfTagSize)],  ... p0
    [(p4X + fullTagSize); p4Y],                     ... p1
    [(p4X + fullTagSize);(p4Y + fullTagSize)],      ... p2
    [p4X; (p4Y + fullTagSize)],                     ... p3
    [p4X; p4Y]];                                    ... p4
    %toc
%% MY IMPLEMENTATION END --------------------------------------------------
end