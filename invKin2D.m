function theta = invKin2D(l, theta0, pos, n, mode)
	% initialize variables
	theta = theta0;
	threshold = 1e-3; % when pos_diff less than this, is converged.
	dPos = [Inf;Inf];
	k = 1
	% choose the method
	if mode == 1
		% use Newton's method
		while( k<n && norm(dPos)>threshold)
			[theta, dPos] = newtonIter(l, theta, pos);
		end
	elseif mode == 0
		[current_pos, J] = evalRobot2D(l, theta);
		B = J;	% initialize B to J, based on guess theta_0;
		while( k<n && norm(dPos)>threshold)
			[theta, dPos, B] = broydenIter(l, theta, pos, B);	
			% update theta and B, recalculate the difference each iter.
		end
	end
	% check if the result converges
	if norm(dPos) >= threshold
		warning('invKin2D did not converge below threshold of %f after %d iterations', threshold, n);
	end
end

%% note:
%%	we are looking for to solve not f(x) = 0, rather, solving f(x)=y;
%%	but we can use the same newton and broyden method still
%%	
%%	let g(x) = f(x) - y
%%	let x_k be that f(x_k) = y, which is the solution
%%	then g(x_k) = 0;
%%	solving for root for g(x)=0 would yield the result for f(x)=y;
%%	furthermore, since y is just a constant
%%	g'(x) == f'(x)
%%	therefore we can use the same Jacobian.

function [theta, diff] = newtonIter(l, theta, pos)
%% use newton's method to find theta
%% this only performs exactly 1 iteration.

	% find the current position and J (derivative)
	% based on the initial guess
	[calc_pos, J] = evalRobot2D(l, theta);
	diff = calc_pos - pos;
	delta_theta = -J\diff;
	theta = theta + delta_theta;
end

function [theta, f_val, B] = broydenIter(l, theta, pos, B)
%% use Broyden's method to find theta
%% only perfomrs 1 iteration

	calc_pos = getEFPosition2D(l, theta);
	f_val = calc_pos - pos; % this is the value from f(x_0),
	% for which we are looking for x that f(x) = 0;
	% solve for step using current B;
	delta_theta = -B\f_val;
	theta = theta + delta_theta;

	% recalculate f(x)
	y = getEFPosition2D(l, theta) - calc_pos;	% f(theta + delta_theta) - f(theta)
	B = B + ((y - B*delta_theta)* delta_theta') / (delta_theta' * delta_theta);
end

function pos = getEFPosition2D(l,theta)
%% those are local functions that are used only in this file
%% use the given formulas:
%% f1(𝜃1, 𝜃2) = l1 cos(𝜃1) + l2 cos(𝜃1 + 𝜃2) - x = 0,
%% f2(𝜃1, 𝜃2) = l1 sin(𝜃1) + l2 sin(𝜃1 + 𝜃2) - y = 0.
	l1 = l(1);
	l2 = l(2);
	theta1 = theta(1);
	theta2 = theta(2);
    % evaluate the position of the end effector
    x = cos(theta1) + l2 * cos(theta1 + theta2);
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
    pos = [x; y];
end
