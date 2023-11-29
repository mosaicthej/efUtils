function J = numerJacobian2D(F, X, h)
	% computes Jacobian matrix numerically for a given function F
	% X is the input vector, h is the perturbation
	
	n = length(X)	% get input size.
	m = 2 % get output size.

	J = zeros(m, n) % m rows (each f_k), n cols (each x_i)
	
	% calculate partial derivative for each part

	for i = 1:n
		dx = zeros(n, 1);
		dx(i) = h;
		% dx = [0, 0, ...., h, 0, 0] ^T
		y_plus = F(X + dx);
		y_minus = F(X - dx);

		
		J(:, i) = (y_plus - y_minus) / (2*h);
	end
end

function pos = getEFPositionTemp(theta)
	global l1;
	global l2;
	theta1 = theta(1);
	theta2 = theta(2);
    % evaluate the position of the end effector
    x = cos(theta1) + l2 * cos(theta1 + theta2);
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
    pos = [x; y];
    clear global;
end
