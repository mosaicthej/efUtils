function theta = invKin2D(l, theta0, pos, n, mode)
	% initialize variables
	theta = theta0;
	threshold = 1e-3; % when pos_diff less than this, is converged.
	dPos = [Inf;Inf];
	k = 0;
%% redefine f as the difference between guessed pos - pos
	g = @(l, theta) [l(1)*cos(theta(1)) + l(2)*cos(theta(1) + theta(2));
		    l(1)*sin(theta(1)) + l(2)*sin(theta(1) + theta(2))];

	f = @(theta) g(l, theta) - pos;
	% choose the method
	if mode == 1
		% use Newton's method
		while( k<n && norm(dPos)>threshold)
			[theta, dPos] = newtonIter(l, theta, pos);
			k=k+1;
		end
	elseif mode == 0
		% initialize J, based on guess theta_0;
		[~,J] = evalRobot2D(l, theta);
		Fx0 = f(theta);
		while(k<n && norm(Fx0)>threshold)
			dTheta = -J\Fx0;
			theta = theta + dTheta;
			Fx = f(theta);
			J = J + ((Fx-Fx0 - J*dTheta)*dTheta')/(dTheta' * dTheta);
			Fx0 = Fx;
            k=k+1;

		end
		dPos = Fx0;

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

