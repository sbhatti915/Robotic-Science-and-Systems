% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks) for localization
% Output: x_est -> 1xT cell array containing the vehicle state mean
%                  for T time steps (i.e., x_est{t} is a 3x1 vector)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a 3x3 matrix)
%
% Your code should include something like the following that calculates the 
% predicted range and bearing observation given that our predicted robot
% pose is x.
% 
%             landmark = map.map(:, zind(t));
%             diff = landmark - x(1:2);
%             r = norm(diff);
%             z = [r; angdiff(atan2(diff(2), diff(1)), x(3))];
% 
% 
% 
function [x_est, P_est] = E1(odo, zind, z, V, W, x0, P0, map)

    x = x0;
    P = P0;
    T = length(zind);
    x_est = cell(1,T);
    P_est = cell(1,T);
    

    for t = 1:T

        % YOUR CODE HERE
        Fx = [1, 0, -odo(1,t)*sin(x(3)); 0, 1, odo(1,t)*cos(x(3)); 0, 0, 1];
        Fv = [cos(x(3)),0; sin(x(3)),0; 0,1];
        
        % Prediction Step
        xPred = [x(1) + odo(1,t)*cos(x(3)); x(2) + odo(1,t)*sin(x(3)); x(3) + odo(2,t)];
        pPred = Fx*P*Fx' + Fv*V*Fv';
        
        % Update Step
        if zind(t)

            landmark = map.map(:, zind(t));
            diff = landmark - xPred(1:2);
            zPred = [norm(diff); angdiff(atan2(diff(2), diff(1)), xPred(3))];
            innovation = z{t} - zPred;
            
            Hx = [-diff(1)/norm(diff), -diff(2)/norm(diff), 0;...
                diff(2)/((norm(diff))^2), -diff(1)/((norm(diff))^2), -1];
            Hw = [1,0;0,1];
            
            S = Hx*pPred*Hx' + Hw*W*Hw';
            kalmGain = pPred*Hx'*inv(S);
            
            x = xPred + kalmGain*innovation;
            P = pPred - kalmGain*Hx*pPred;
        else
            x = xPred;
            P = pPred;
        end
        x_est{t} = x;
        P_est{t} = P; 
    end

end