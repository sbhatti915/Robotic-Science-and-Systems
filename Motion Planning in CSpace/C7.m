% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

%Use with 300 resolution
function padded_cspace = C7(cspace)
    
    padded_cspace = cspace;
    
    neighbors = [-1 -1 -1 0 0 1 1 1; -1 0 1 -1 1 -1 0 1];
    
    for i = 2:length(cspace) - 1
        for j = 2:length(cspace) - 1
            if cspace(i,j) == 1
                for k = 1:length(neighbors)
                    xInd = i + neighbors(1,k);
                    yInd = j + neighbors(2,k);
                    if cspace(xInd, yInd) == 0
                        padded_cspace(xInd, yInd) = 1;
                    end
                end
            end
            
        end
    end
end


