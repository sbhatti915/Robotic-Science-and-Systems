% Fit a plane using RANSAC
% input        P ->  3x100 matrix denoting 100 points in 3D space
% output: normal -> 1x3 vector denoting surface normal of the fitting plane
%         center -> 1x3 vector denoting center of the points
function [normal,center] = Q1_c(P)
    threshold = 0.001;
    lastInliers = 0;
    num_iterations = 10000;
    

    for j = 1:num_iterations
        inliers = 0;
        
        % Sample random points
        P1 = P(:,randi(100));
        P2 = P(:,randi(100));
        P3 = P(:,randi(100));
        
        % Eliminate possibility where P1 can equal P2 and/or P3 and vice versa
        while (P1 == P2) | (P1 == P3) | (P2 == P3)
            P1 = P(:,randi(100));
            P2 = P(:,randi(100));
            P3 = P(:,randi(100));
        end
            
        newP = [P1,P2,P3];
        
        cent = mean(newP,2)';
        
        covariance = cov((newP-cent')');
        
        % Find eigenvectors and eigenvalues
        [vec,val] = eig(covariance);
        for j = 1:length(val)
            eigenval(j) = val(j,j);
        end
        
        [~,ind] = min(eigenval);
        
        nor = vec(:,ind)';
        
        for k = 1:length(P)
            distance = abs(dot((P(:,k))' - cent, nor));
            
            if distance < threshold
                inliers = inliers + 1;
            end
        end
        
        if inliers > lastInliers
            lastInliers = inliers;
            center = cent;
            normal = nor;
        end
    end
        
end
