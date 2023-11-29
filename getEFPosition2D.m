% those are local functions that are used only in this file
% use the given formulas:
% f1(𝜃1, 𝜃2) = l1 cos(𝜃1) + l2 cos(𝜃1 + 𝜃2) - x = 0,
% f2(𝜃1, 𝜃2) = l1 sin(𝜃1) + l2 sin(𝜃1 + 𝜃2) - y = 0.
function pos = getEFPosition2D(l,theta)
	l1 = l(1);
	l2 = l(2);
	theta1 = theta(1);
	theta2 = theta(2);
    % evaluate the position of the end effector
    x = cos(theta1) + l2 * cos(theta1 + theta2);
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
    pos = [x; y];
end
