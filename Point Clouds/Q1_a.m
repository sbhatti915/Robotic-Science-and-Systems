% Fit a least squares plane by taking the Eigen values and vectors of the
% sample covariance matrix.
% input: P       ->  3x100 matrix denoting 100 points in 3D space
% output: normal -> 1x3 vector denoting surface normal of the fitting plane
%         center -> 1x3 vector denoting center of the points
function [normal,center] = Q1_a(P)
    center = mean(P,2)';
    
    % Find covariance matrix
    covariance = cov((P-center')');
    
    % Find eigenvectors and eigenvalues
    [vec,val] = eig(covariance);
    for k = 1:length(val)
        eigenval(k) = val(k,k);
    end
    
    [~,ind] = min(eigenval);
    
    normal = vec(:,ind)';
    
end
