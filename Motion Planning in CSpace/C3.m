% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    distances = cspace;

    neighbors = [-1 -1 -1 0 0 1 1 1; -1 0 1 -1 1 -1 0 1];
    
    [~,goalx_Ind] = min(abs(q_grid - q_goal(1)));
    [~,goaly_Ind] = min(abs(q_grid - q_goal(2)));
    distances(goalx_Ind, goaly_Ind) = 2;
   
    frontier = [goalx_Ind, goaly_Ind];
    
    while size(frontier,1) > 0
        
        exploreDist = distances(frontier(1,1), frontier(1,2));
        
        for k = 1:length(neighbors)
            
            nextxInd = frontier(1,1) + neighbors(1,k);
            nextyInd = frontier(1,2) + neighbors(2,k);
            
            if ((nextxInd > 0) && (nextyInd > 0) && (nextxInd < length(cspace)) && (nextyInd < length(cspace)) && distances(nextxInd,nextyInd) == 0)
                
                frontier(size(frontier,1)+1,:) = [nextxInd, nextyInd];
                distances(nextxInd, nextyInd) = exploreDist + 1;
                
            end
        end
        frontier(1,:) = [];
    end
end