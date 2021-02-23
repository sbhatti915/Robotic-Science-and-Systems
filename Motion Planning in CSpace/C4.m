% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)    
    neighbors = [-1 -1 -1 0 0 1 1 1; -1 0 1 -1 1 -1 0 1];
    
    [~,goalx_Ind] = min(abs(q_grid - q_start(1)));
    [~,goaly_Ind] = min(abs(q_grid - q_start(2)));
    path = [goalx_Ind, goaly_Ind];
    
    while distances(path(end,1), path(end,2)) > 2 || distances(path(end,1), path(end,2)) < 2
        xInd = path(end,1);
        yInd = path(end,2);
        
        currentDist = distances(xInd,yInd);
        
        for k = 1:length(neighbors)
            
            nextxInd = path(end,1) + neighbors(1,k);
            nextyInd = path(end,2) + neighbors(2,k);
            dist = distances(nextxInd,nextyInd);
            
            if dist <= 1
                continue
            end
            
            if dist < currentDist
                currentDist = dist;
                xInd = nextxInd;
                yInd = nextyInd;
            elseif dist == currentDist % First come, first serve tie breaker
                continue
            end
        end
        path(end+1,:) = [xInd,yInd];
    end
end