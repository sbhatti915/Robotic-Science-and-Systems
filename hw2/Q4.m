% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory. The end effector should trace a circle in the
%                 workspace, as specified by the input argument circle.
%                 (orientation is to be ignored)

function traj = Q4(f, qInit, circle, velocity)
    % Declarations
    epsilon = 0.01; % scalar of how close end effector must be before loop terminates
    traj(1,:) = qInit; % Initialize first configuration of trajectory
    k = 1; % ending row of trajectory
    
    % Find trajectory that moves end effector in a circle at constant velocity
    for i = 1:length(circle) % Count for number of columns in circle
        posGoal = circle(:,i); % Set target = to each 3x1 position in circle
        trajStep = Q3(f, traj(k,:), posGoal, epsilon, velocity); % Calculate new configurations (trajectory) given new target from circle and traj calculated through Q3 function
        stepsize = size(trajStep,1); % Find number of new configurations in trajStep
        k = k - 1 + stepsize; % Update the last row with the number of new configurations
        
        traj(k+1-stepsize:k,:) = trajStep; % Append trajectory matrix with new configurations calculated from trajStep
    end
    %%% See Q3 for speed velocity check
end