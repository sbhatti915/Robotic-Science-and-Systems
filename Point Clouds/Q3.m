% Localize a cylinder in the point cloud. Given a point cloud as input, this
% function should locate the position and orientation, and radius of the
% cylinder.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting cylinder center
%         axis -> 3x1 unit vector pointing along cylinder axis
%         radius -> scalar radius of cylinder
function [center,axis,radius] = Q3(ptCloud)
    % Declarations
    threshold = 0.001;
    lastInliers = 0;
    num_iterations = 10000;
    
    P = ptCloud.Location;
    
    norms = pcnormals(ptCloud,15);
    
    % Algorithms
    for k = 1:num_iterations
        inliers = 0; % Reset number of inliers
        
        % Random indices
        randInd1 = randi(size(P,1));
        randInd2 = randi(size(P,1));
        
        while randInd1 == randInd2
            randInd2 = randi(size(P,1));
        end
        
        % Sample points
        pt1 = P(randInd1,:);
        pt2 = P(randInd2,:);
        
        % Estimate Surface Normal of points
        ptNorm1 = norms(randInd1,:);
        ptNorm2 = norms(randInd2,:);
        
        % Find cylinder axis
        ax = cross(ptNorm1,ptNorm2)';
        
        rad = 0.05 + rand(1,1)*(0.10 - 0.05); % Sphere is between radius 0.10m and 0.05m
        
        % Center of circle
        cent = pt1 + rad*ptNorm1;
        
        % Projections
        plane = (eye(3) - ax*ax')*P';
        centPlane = (eye(3) - ax*ax')*cent';
        
        % Find inliers
        distance = pdist2(plane',centPlane','Euclidean');
        inliers = sum(abs(distance-rad) < threshold);
        
        if inliers > lastInliers
            lastInliers = inliers;
            center = cent';
            radius = rad;
            axis = ax;
        end
    end
end