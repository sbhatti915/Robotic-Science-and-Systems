% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector describing goal position
%        epsilon -> scalar denoting how close end effector must be before
%                   loop terminates
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory.

function traj = Q3(f, qInit, posGoal, epsilon, velocity)
    % Declarations
    q = qInit;
    traj(1,:) = q; % First row of traj is the initial configuration
    x = f.fkine(q); % Find forward kinematics
    
    k = 2; % Initialize Counter
    
    while norm(posGoal - x.t) > epsilon
        x = f.fkine(q); % Calculate forward kinematics, 4x4 T matrix
        Jinv = pinv(f.jacob0(q)); % Calculate pseudoinverse of Jacobian
        dx = posGoal-x.t; % Difference between goal position and current position
        dx = dx/(norm(dx)); 
        dq = velocity*Jinv(:,1:3)*dx(1:3); % Find change of joint angles (ignoring orientation)
        q = q + dq'; % Add change
       
        traj(k,:) = q; % Append traj with new configurations
        
%         velocityCheck = norm(f.fkine(traj(k,:))-f.fkine(traj(k-1,:))); % Checks velocity to make sure it is equal to input velocity
%         disp(velocityCheck)

        k = k + 1; % Add 1 to counter
    end
end