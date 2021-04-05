% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    samples = zeros(num_samples, 4);
    adjacency = zeros(num_samples);

    for i = 1:num_samples
        qs = M1(q_min, q_max, num_samples); % Constantly generate new samples
        k = randi(num_samples); % Pick random number between 1 and num_samples
        while check_collision(robot, qs(k,:), link_radius, sphere_centers, sphere_radii)
            k = randi(num_samples); % Pick new random number between 1 and num_samples if collision
        end
        samples(i,:) = qs(k,:);
    end

    % Check if near points can be connected.

    for j = 1:num_samples
        
        distances = vecnorm(samples - samples(j,:),2,2);
        
        [B, I] = sort(distances);
        
        neighbors = I(2:num_neighbors+1,:);
        dist = B(2:num_neighbors+1,:);

        for k = 1:length(dist)
            
            if ~check_edge(robot, samples(j,:), samples(neighbors(k),:), link_radius, sphere_centers, sphere_radii)
                
                adjacency(j,neighbors(k)) = dist(k);
                adjacency(neighbors(k),j) = dist(k);
                
            end
            
        end
    end
end

