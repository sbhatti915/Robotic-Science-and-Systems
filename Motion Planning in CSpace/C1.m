% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to plot the robot at

function C1(robot, q)
    % Find shapes and pivots points of robot
    [poly1, poly2, piv1, piv2] = q2poly(robot, q);

    % Plot the links
    plot(poly1, 'FaceColor', 'r');
    plot(poly2, 'FaceColor', 'b');
    
    % Plot the pivot points
    plot(piv1(1), piv1(2), 'k.', 'MarkerSize', 10);
    plot(piv2(1), piv2(2), 'k.', 'MarkerSize', 10);

end