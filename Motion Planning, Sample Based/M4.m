% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    beta = 0.1;
    alpha = 0.3;
    sample_size = 500;
    V = q_start;
    E = [];
    

    for k = 1:sample_size

        if rand(1) <= beta
            q_target = q_goal;
        else
            q_target = M1(q_min, q_max, 1);
        end
        
        I = knnsearch(V,q_target);
        q_near = V(I,:);
        
        q_new = q_near + alpha*(q_target-q_near)/norm(q_target-q_near);
        
        if ~check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii) && ~check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii)

            V = [V; q_new];
            E = [E; I,length(V)];
        end
    end

    path = [];
    
    G = digraph(E(:,1),E(:,2));
    
    [neighborInd,~] = knnsearch(V,[q_start;q_goal],'K',1);
    
    [path,dist] = shortestpath(G,neighborInd(1,1),neighborInd(2,1));
    
    if isinf(dist)
        path_found = false;
    else
        path_found = true;
    end
    
    path = V(path,:);
    
    path = [q_start;path];
    path = [path;q_goal];
    
    
        
