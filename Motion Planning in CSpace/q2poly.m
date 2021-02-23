% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    %rotq = rot2(q);
    
    %rotq1(1,:) = rotq(1,:);
    %rotq1(2,:) = rotq(3,:);
    
    %rotq2(1,:) = rotq(2,:);
    %rotq2(2,:) = rotq(4,:);
    
    q1 = q(1);
    q2 = q(2);
    
    rotq1 = rot2(q1);

    rotq2 = rot2(q2);
    
    % Find pivot points
    pivot1 = robot.pivot1;
    pivot2 = rotq1*robot.pivot2 + pivot1;
    
    link1 = robot.link1;
    link2 = robot.link2;
    
    % Find rotation of links
    for k = 1:length(link1)
        link1(:,k) = rotq1*link1(:,k);
        link2(:,k) = rotq2*rotq1*link2(:,k);
    end
    
    % Add links to pivot points
    link1 = link1 + pivot1;
    link2 = link2 + pivot2;
    
    poly1 = polyshape(link1(1,:), link1(2,:));
    poly2 = polyshape(link2(1,:), link2(2,:));
end