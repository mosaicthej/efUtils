function theta = invKin3D(l, theta0, desired, n, mode)
	% initialize variables
	theta = theta0;
	threshold = 1e-3; % when pos_diff less than this, is converged.
	dPos = [Inf; Inf; Inf]; % Initialize position difference
	k = 0;
%% redefine f as the difference between guessed pos - pos
	% Define the function for forward kinematics
    g = @(theta) evalRobot3D(l, theta);

    % Define the error function as the difference between current and desired positions
    f = @(theta) g(theta) - desired;
	% choose the method
	if mode == 1
		% use Newton's method
		while( k<n && norm(dPos)>threshold)
			[theta, dPos] = newtonIter(l, theta, desired);
			k=k+1;
		end
	elseif mode == 0
		% initialize J, based on guess theta_0;
		[pos,B] = evalRobot3D(l, theta);
		Fx0 = pos - desired;
		while(k<n && abs(norm(Fx0))>threshold)
			dTheta = -B\Fx0;
			theta = theta + dTheta;
			Fx = f(theta);
			B = B + ((Fx-Fx0 - B*dTheta)*dTheta')/(dTheta' * dTheta);
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
	[calc_pos, J] = evalRobot3D(l, theta);
	f_val = calc_pos - pos;
	delta_theta = -J\f_val;
	theta = theta + delta_theta;
end


