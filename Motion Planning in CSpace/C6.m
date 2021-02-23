% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)

    num_collisions = 0;
    
    for j = 1:size(q_path,1) - 1 
        [poly1, poly2, ~, ~] = q2poly(robot, q_path(j,:));
        [poly3, poly4, ~, ~] = q2poly(robot, q_path(j+1,:));
        uniPoly1 = union(poly3, poly1);
        uniPoly2 = union(poly4, poly2);
        
        poly1Conv = convhull(uniPoly1);
        poly2Conv = convhull(uniPoly2);
        
        for k = 1:length(obstacles)
            
            if not(isempty(intersect(poly1Conv, obstacles(k)).Vertices) && isempty(intersect(poly2Conv,obstacles(k)).Vertices))

                C1(robot,q_path(j,:))
                C1(robot,q_path(j+1,:))
                
                plot(poly1Conv,'FaceColor', 'r');
                plot(poly2Conv,'FaceColor', 'b');
                
                num_collisions = num_collisions + 1;
            end
        end
    end
end
