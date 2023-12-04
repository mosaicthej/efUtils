% Define the lengths of the manipulator's links
l = [0.5; 0.5]; % For example, two links of length 1

theta_test = [pi/2;pi/4;pi/3];
target = evalRobot3D(l,theta_test);
if isrow(target)
	target = target';
end
% Define a target point near the end effector, use expected from theta. 

% Initial guess for the joint angles theta0
theta0 = theta_test * 0.9; % Assume starting from somewhere near
% avoid of 0 to get singular Jacobian

% Test the algorithm with Newton's method
n_iterations = 1000;
mode = 1; % Newton's method
% initially test if the Newton puts to the correct theta
theta = invKin3D(l, theta0, target, n_iterations, mode);
calc_pos = evalRobot3D(l,theta); 
if isrow(target) target=target'; end
fprintf("difference between calculated theta and expected is \n");
disp(calc_pos - target);

%% Q2.2.a.1
%% Testing when position is linearly increasing.
% Check convergence for a point moving away from the end effector
disp('Testing convergence as the target moves away:');
dist = 1;
while norm(target*dist)<= sum(l) % Increase the distance from 1 to 2 in steps of 0.1
    % Scale the target point
    scaled_target = target *dist;
    % Try to find the joint angles to reach the scaled target
    theta = invKin3D(l, theta0, scaled_target, n_iterations, mode);
    % Check if the algorithm converged
    [pos, ~] = evalRobot3D(l, theta);
    if isrow(pos) pos=pos'; end
    error_norm = norm(pos - scaled_target);
    fprintf('Distance: %f, Error norm: %f\n', dist, error_norm);
    if error_norm > 1e-3
        disp('Newton''s method stopped converging.');
        % Explain why it stops converging
        disp('This may be due to reaching a singularity, poor initial guess,');
        disp('or the target being outside the workspace of the manipulator.');
        break;
    end
	dist=dist+0.01;
end

%% Q2.2.a.2
%% Testing when angles are moving away...
% Check convergence for a point moving away from the end effector
disp('Testing convergence as the target moves away:');
converged=true;
% try to find the angles that will cause Newton fail to converge.
% since each theta is -pi< theta < pi;
% traverse the D3 vector space for all possible theta as expected place
theta_step=100;
theta0 = [-pi/2;-pi/2;-pi/2]; % starting point
converged = true;
i=-pi/2;
while (i<pi && converged)
	j=-pi/2;
	while (j<pi && converged)
		if abs(j)<1e-3
			continue
		end
		k=-pi/2;
		while (k<pi && converged)
			target_theta=[i;j;k];
			[target_pos, ~] = evalRobot3D(l, target_theta);
			if isrow(target_pos) target_pos=target_pos';end
			calc_theta=invKin3D(l, theta0, target_pos, n_iterations, mode); % finding the theta
			[calc_pos, ~] = evalRobot3D(l, calc_theta);
			if isrow(calc_pos) calc_pos=calc_pos';end
			error_norm = norm(target_pos-calc_pos);
			fprintf('angles: %f %f %f, Error norm: %f\n', i, j, k, error_norm);
			if (error_norm > 1e-3) || (isnan(error_norm))
				% now, Newton's method stopped converging.
				% because the initial guess is too far away from the actual angle.
				% or could also be that Jacobian becomes singular during the calculation.
				disp('Newton''s method stopped converging.');
				% Explain why it stops converging
				disp('This may be due to reaching a singularity, poor initial guess,');
				disp('or the target being outside the workspace of the manipulator.');
				converged=false;
			end
			k=k+(2*pi)/theta_step;
		end
		j=j+(2*pi)/theta_step;
	end
	i=i+(2*pi)/theta_step;
end
			
%% Q2.2.b
% Tackling the problem where the target is far from the end effector
% - One approach is to generate a series of intermediate points between
%   the current end effector position and the distant target.
% - This can be done by interpolating a straight line or a smooth curve.
% - At each intermediate point, apply the inverse kinematics solver.
% - This incremental approach can help to avoid singularities and ensure
%   a smoother transition between configurations.

%% Q2.2.c
% Now, choose a target point in the diametrically opposite quadrant
theta0 = [pi/2;pi/3;pi];
target = evalRobot3D(l, theta0);
if isrow(target) target=target'; end
opposite_target = [-target(1); -target(2); -target(3)];
disp('Trying to reach a point in the opposite quadrant:');
theta = invKin3D(l, theta0, opposite_target, n_iterations, mode);
[pos, ~] = evalRobot3D(l, theta);
if isrow(pos) pos=pos'; end
error_norm = norm(pos - opposite_target);
fprintf('Opposite target Error norm: %f\n', error_norm);
direct_fail = error_norm > 1e-3;
% If the algorithm fails to reach the opposite target directly,
% plan a path with intermediate points to guide the solution.
% This approach can avoid configurations that result in a singular Jacobian
% and can be more likely to find a valid solution to the inverse kinematics.
if direct_fail
%if true
% it failed because the initial guess is too far compares to the solution
% therefore Newton's method failed to converge.
% in this case, newton failed to converge to opposite target.
	disp("Direct attempt failed to converge, Plan a path with Bezier");
	% define start and end points
	pos_start = evalRobot3D(l, theta0);
	if isrow(pos_start) pos_start=pos_start'; end
	pos_end = opposite_target;
	
	% define control points for Bezier
	control_points = [pos_start, pos_start+0.5*(pos_end-pos_start), pos_end];

	% generate intermediate path along the curve
	t_values = linspace(0,1,10);
	bezier_path = zeros(3, length(t_values));
	for i = 1:length(t_values)
		t = t_values(i);
		bezier_path(:,i) =...
			(1-t)^2*control_points(:,1)+...
			2*(1-t)*t*control_points(:,2)+...
			t^2*control_points(:,3);
	end

	% now for each point along the bezier path
	for i = 1:size(bezier_path, 2)
		intermediate_target = bezier_path(:,i);
		theta = invKin3D(l, theta, intermediate_target, n_iterations, mode);
		[pos,~] = evalRobot3D(l, theta);
		if isrow(pos) pos=pos'; end
		error_norm = norm(pos-intermediate_target);

		% check if intermediate solution is successful;
		if error_norm > 1e-3
			fprintf("Failed to converge at intermediate point %d\n", i);
		else
			fprintf("Beizer curve at %f %f %f is successful\n",...
				intermediate_target(1), ...
				intermediate_target(2), ...
				intermediate_target(3));
		end
	end
	
	% Check for final converge
	final_error_norm = norm(pos-pos_end);
	if final_error_norm < 1e-3
		disp('successfully reached the opposite quadrant target using Bezier');
	else
		disp('Failed to reach the opposite quadrant using Bezier');
	end
else
	disp('Direct method worked for opposite quadrant');
end
