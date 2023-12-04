% Define the lengths of the manipulator's links
l = [0.5; 0.5]; % For example, two links of length 1
% >> evalRobot3D([0.5,0.5],[0.1,0.2,0.3])  
% ans =
%     0.9316    0.2882    0.1977
%
% testing scenario: when l=[0.5;0.5], theta=[0.1;0.2;0.3]
% the correct position is [0.9316; 0.2882; 0.1977]

% Initial guess for the joint angles theta0
theta0 = [0.2; 0.3; 0.4]; % Assume starting in the default position
% avoid of 0 to get singular Jacobian

% Define a target point near the end effector, use the expected from above
target = [0.9316; 0.2882; 0.1977]; 
% expected calculated thetas will be [0.1;0.2;0.3];

% Test the algorithm with Newton's method
n_iterations = 100;
mode = 1; % Newton's method

% Check convergence for a point moving away from the end effector
disp('Testing convergence as the target moves away:');
for dist = 1:0.1:2 % Increase the distance from 1 to 2 in steps of 0.1
    % Scale the target point
    scaled_target = target * dist;
    % Try to find the joint angles to reach the scaled target
    theta = invKin3D(l, theta0, scaled_target, n_iterations, mode);
    % Check if the algorithm converged
    [pos, ~] = evalRobot3D(l, theta);
    error_norm = norm(pos - scaled_target);
    fprintf('Distance: %f, Error norm: %f\n', dist, error_norm);
    if error_norm > 1e-3
        disp('Newton''s method stopped converging.');
        % Explain why it stops converging
        disp('This may be due to reaching a singularity, poor initial guess,');
        disp('or the target being outside the workspace of the manipulator.');
        break;
    end
end

% Tackling the problem where the target is far from the end effector
% - One approach is to generate a series of intermediate points between
%   the current end effector position and the distant target.
% - This can be done by interpolating a straight line or a smooth curve.
% - At each intermediate point, apply the inverse kinematics solver.
% - This incremental approach can help to avoid singularities and ensure
%   a smoother transition between configurations.

% Now, choose a target point in the diametrically opposite quadrant
opposite_target = [-target(1); -target(2); -target(3)];
disp('Trying to reach a point in the opposite quadrant:');
theta = invKin3D(l, theta0, opposite_target, n_iterations, mode);
[pos, ~] = evalRobot3D(l, theta);
error_norm = norm(pos - opposite_target);
fprintf('Opposite target Error norm: %f\n', error_norm);

% If the algorithm fails to reach the opposite target directly,
% plan a path with intermediate points to guide the solution.
% This approach can avoid configurations that result in a singular Jacobian
% and can be more likely to find a valid solution to the inverse kinematics.

% Optional: To make the motions smoother, consider using splines or Bezier
% curves to create the path. This would require generating a sequence of
% target points along the curve and applying the inverse kinematics solver
% to each point in sequence, resulting in a smooth trajectory.

