function theta = invKin2D(l, theta0, pos, n, mode)
	% initialize variables
	theta = theta0;
	threshold = 1e-3; % when pos_diff less than this, is converged.
	dPos = [Inf;Inf];
	k = 0;
	% choose the method
	if mode == 1
		% use Newton's method
		while( k<n && norm(dPos)>threshold)
			[theta, dPos] = newtonIter(l, theta, pos);
			k=k+1;
		end
	elseif mode == 0
		[~, J] = evalRobot2D(l, theta);
		B = J;	% initialize B to J, based on guess theta_0;
		while( k<n && norm(dPos)>threshold)
			[theta, dPos, B] = broydenIter(l, theta, pos, B);	
			% update theta and B, recalculate the difference each iter.
			k=k+1;
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

function [theta, f_val] = newtonIter(l, theta, pos)
%% use newton's method to find theta
%% this only performs exactly 1 iteration.

	% find the current position and J (derivative)
	% based on the initial guess
	[calc_pos, J] = evalRobot2D(l, theta);
	f_val = calc_pos - pos;
	delta_theta = -J\f_val;
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
	y = getEFPosition2D(l, theta) - pos;	% f(theta + delta_theta) - f(theta)
%	B = B + ((y - B*delta_theta)* delta_theta') / (delta_theta' * delta_theta);
	B = B + (y - f_val - B*delta_theta) / dot(delta_theta, delta_theta)*delta_theta.';
	f_val = y;
end
