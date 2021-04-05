% Localize a sphere in the point cloud. Given a point cloud as input, this
% function should locate the position and radius of a sphere.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
function [center,radius] = Q2(ptCloud)

    % Declarations
    threshold = 0.001;
    lastInliers = 0;
    num_iterations = 10000;
    
    P = ptCloud.Location;
    
    norms = pcnormals(ptCloud);
    
    % Algorithm
    for j = 1:num_iterations
        inliers = 0; % Reset number of inliers
        randInd = randi(size(P,1));
        
        % Sample a point
        pt = P(randInd,:,:);
        
        % Estimate Surface Normal of point
        ptNorm = norms(randInd,:);
        
        rad = 0.05 + rand(1,1)*(0.11 - 0.05); % Sphere is between radius 0.11m and 0.05m
        
        cent = pt + rad*ptNorm;
        
        distance = pdist2(P,cent,'Euclidean');
        
        inliers = sum(abs(distance-rad) < threshold);
        
        if inliers > lastInliers
            lastInliers = inliers;
            center = cent';
            radius = rad;
        end
    end
end