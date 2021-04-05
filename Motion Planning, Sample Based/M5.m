% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    k = 1;
    smoothed_path = [path(1,:)];
    while k < length(path)
        for j = length(path):-1:k
            if ~check_edge(robot, path(k,:), path(j,:), link_radius, sphere_centers, sphere_radii)
                smoothed_path = [smoothed_path;path(j,:)];
                k = j;
                break
            end
        end
    end      
end