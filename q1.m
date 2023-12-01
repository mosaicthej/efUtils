% Q1.1 
l = [1;1]; % length
theta = [pi/4;pi/4]; % angle
[pos, J] = evalRobot2D(l, theta);

disp('Position of the End Effector:');
disp(pos);
disp('Jacobian:');
disp(J);

% Q1.2
disp('now calculating with Numerical Jacobian using h=');
h=1E-4

J_n = fdJacob2D(l, theta, h);

disp(J_n);

disp('difference is');
J - J_n

% a. are the results close enough to be useful?

% Yes the results are very close from numerical calculation
% 	and finding the Jacobian analytically.
%
% In fact, the differences are usually in the order of h^2,
% 	(e.g. when h = 1E-3, the difference are on the order of 1E-6)
%	Therefore, we can yield a very close approximation of the Jacobian
%	without too small h.
%
% This makes the approach very useful


% b. Why would you use this finite-difference approximation instead of 
%	the analytic derivative?
%
% Usually, we do not know the equation to determine the end-effector beforehand
%	even when we do, the equation are too complex to solve for derivative
%	analytically.
% Also even if we have the equation, it's usually not accurate in the
%	real-world due to factors like frictions and air resistance.
% 
% Lastly, once we have the algorithm for finite-difference, we can take in
%	any formulas for calculation, whereas with analytic derivative,
%	the derivatives needs to be calculated for each formula.

% 
% Q1.4
% Newton's method generally converges faster than Broyden's method.
% Broyden's method is easier to implement when the derivative of the function is unknown or difficult to compute.
% Broyden's method is more computationally efficient, especially for complex functions where calculating derivatives is costly.
% Newton's method can be more sensitive to the initial guess and may fail to converge in some cases, 
% especially in the presence of discontinuities or non-smooth functions.



