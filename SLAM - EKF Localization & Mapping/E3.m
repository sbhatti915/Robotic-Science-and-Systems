% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first
%                    three rows of x_est and P_est correspond to the
%                    vehicle state, the next two correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E3(odo, zind, z, V, W, x0, P0)

    x = x0;
    P = P0;
    indices = [];
    T = length(zind);
    x_est = cell(1,T);
    P_est = cell(1,T);

    for t = 1:T

        % YOUR CODE HERE
        x = [x(1) + odo(1,t)*cos(x(3)); x(2) + odo(1,t)*sin(x(3)); x(3) + odo(2,t); x(4:end)];
        wid = size(x);
        Fx = eye(wid(1));
        Fx(1:3,1:3) = [1, 0, -odo(1,t)*sin(x(3)); 0, 1, odo(1,t)*cos(x(3));0, 0, 1];
        
        Fv = zeros(wid(1),2);
        Fv(1:3,1:2) = [cos(x(3)),0; sin(x(3)),0; 0,1];
        
        P = Fx*P*Fx' + Fv*V*Fv';
        if zind(t)
            if ismember(zind(t),indices) % Old Landmark
                k = find(indices == zind(t));

                diff = [x(2*k+2); x(2*k+3)] - x(1:2);
                zPred = [norm(diff); angdiff(atan2(diff(2), diff(1)), x(3))];

                innovation = z{t} - zPred;

                
                Hpi = [diff(1)/norm(diff), diff(2)/norm(diff); -diff(2)/((norm(diff))^2), diff(1)/((norm(diff))^2)];
                Hxv = [-diff(1)/norm(diff), -diff(2)/norm(diff), 0; diff(2)/((norm(diff))^2), -diff(1)/((norm(diff))^2), -1];
                
                Hx = [Hxv, zeros(2,2*k-2), Hpi, zeros(2,2*length(indices)-2*k)];
                Hw = [1,0;0,1];

                S = Hx*P*Hx' + Hw*W*Hw';
                kalmGain = P*Hx'*inv(S);

                x = x + kalmGain*innovation;
                P = P - kalmGain*Hx*P;

            else % New landmark
                dummy = z{t};
                g = [x(1) + dummy(1)*cos(x(3) + dummy(2)); x(2) + dummy(1)*sin(x(3) + dummy(2))];
                x = [x;g];

                Gx = [1, 0, -dummy(1)*sin(x(3) + dummy(2));0, 1, dummy(1)*cos(x(3) + dummy(2))];
                Gz = [cos(x(3) + dummy(2)), -dummy(1)*sin(x(3) + dummy(2)); sin(x(3) + dummy(2)), dummy(1)*cos(x(3) + dummy(2))];
                
                Yz = blkdiag(eye(length(P)),Gz);
                Yz(end-1:end,1:3) = Gx;
                P = Yz*blkdiag(P,W)*Yz';
                
                indices = [indices; zind(t)];
         
            end
        end
        x_est{t} = x;
        P_est{t} = P;

    end
end
