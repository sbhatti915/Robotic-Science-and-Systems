% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2(f, qInit, posGoal)
    % Declarations
    q = qInit;
    Target = transl(posGoal); % Creates homogenous matrix using goal position
    alpha = 0.5; % Initialize Scale factor
    
    %Inverse Kinematics 
    for k = 1:100
        
        x = f.fkine(q); % Calculate forward kinematics, 4x4 T matrix
        Jinv = pinv(f.jacob0(q)); % Calculate pseudoinverse of Jacobian
        dx = tr2delta(x,Target); % Find differential motion (6x1) corresponding to infinitessimal motion from pose x to Target
        dq = alpha*Jinv(:,1:3)*dx(1:3); % Find change of joint angles (ignoring orientation)
        q = q + dq'; % Add change
        
    end
end