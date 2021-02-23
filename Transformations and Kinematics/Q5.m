% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First seven joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)

function q = Q5(f1, f2, qInit, f1Target, f2Target)
    % Declarations
    q = qInit;
    alpha = 0.5; % Initialize Scale factor
    
    t1Goal = transl(f1Target); % Creates homogenous matrix using goal 1 position
    t2Goal = transl(f2Target); % Creates homogenous matrix using goal 2 position
    
    %Inverse Kinematics 
    for k = 1:100
        
        % Initialize angles
        q1 = q(:,1:9);
        q2 = [q(:,1:7),q(:,10:11)];
        
        % Find differential motion (6x1) corresponding to infinitessimal
        %   motion from pose f1 to Target goal 1 and pose 2 to Target goal
        %   2. First 3 rows are translation, last 3 are rotation
        dx1 = tr2delta(f1.fkine(q1), t1Goal);     
        dx2 = tr2delta(f2.fkine(q2), t2Goal);    
        
        dx = [dx1; dx2]; % Concatenate into 12x1 matrix dx
        
        % Calculate jacobian of f1
        J1 = f1.jacob0(q1);              
        J1 = [J1(:,1:9), zeros(6,2)]; % Append matrix to be a 6x11 matrix
        
        % Calculate jacobian of f2
        J2 = f2.jacob0(q2);              
        J2 = [J2(:,1:7), zeros(6,2), J2(:,8:9)]; % Append matrix to be a 6x11 matrix
        
        % Concatenate jacobians and calculate inverse
        Jinv = pinv([J1; J2]); % Becomes 11x12 matrix
       
        dq = alpha*Jinv*dx; % Find change of joint angles (11x1)
        q = q + dq'; % Add change (1x11)
        
    end
    %disp(q)
end