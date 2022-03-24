% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the known initial vehicle state
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (2M)x(2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first two
%                    rows of x_est and P_est correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E2(odo, zind, z, W, x0)

    v = x0;
    x = [];
    P = [];
    indices = [];
    T = length(zind);
    x_est = cell(1,T);
    P_est = cell(1,T);

    for t = 1:T

        % YOUR CODE HERE
        v = [v(1) + odo(1,t)*cos(v(3)); v(2) + odo(1,t)*sin(v(3)); v(3) + odo(2,t)];
        if zind(t)
            if ismember(zind(t),indices) % Old Landmark
                k = find(indices == zind(t));

                diff = [x(2*k-1); x(2*k)] - v(1:2);
                zPred = [norm(diff); angdiff(atan2(diff(2), diff(1)), v(3))];

                innovation = z{t} - zPred;

                H = [diff(1)/norm(diff), diff(2)/norm(diff); -diff(2)/((norm(diff))^2), diff(1)/((norm(diff))^2)];
                
                Hx = [zeros(2,2*k-2), H, zeros(2,2*length(indices)-2*k)];
                Hw = [1,0;0,1];

                S = Hx*P*Hx' + Hw*W*Hw';
                kalmGain = P*Hx'*inv(S);

                x = x + kalmGain*innovation;
                P = P - kalmGain*Hx*P;

            else % New landmark
                dummy = z{t};
                g = [v(1) + dummy(1)*cos(v(3) + dummy(2)); v(2) + dummy(1)*sin(v(3) + dummy(2))];
                x = [x;g];

                Gz = [cos(v(3) + dummy(2)), -dummy(1)*sin(v(3) + dummy(2)); sin(v(3) + dummy(2)), dummy(1)*cos(v(3) + dummy(2))];
                Yz = blkdiag(eye(length(P)),Gz);
                P = Yz*blkdiag(P,W)*Yz';
                
                indices = [indices; zind(t)];
         
            end
        end
        x_est{t} = x;
        P_est{t} = P;
    end
    
end