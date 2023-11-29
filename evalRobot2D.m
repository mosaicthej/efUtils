function [pos, J] = evalRobot2D(l, theta)
    % Evaluate the position of the end effector of a 2D robot arm
    % getting the input parameters:
    % l:        vector of the links length
    % theta:    vector of the joints angles
    % for this problem we have degree of freedom = 2
    l1 = l(1);
    l2 = l(2);
    theta1 = theta(1);
    theta2 = theta(2);
    % evaluate the position of the end effector
    pos = getEFPosition(l1, l2, theta1, theta2);

    % evaluate the Jacobian matrix
    J = getJacobian(l1, l2, theta1, theta2);

end

% those are local functions that are used only in this file
% use the given formulas:
% f1(𝜃1, 𝜃2) = l1 cos(𝜃1) + l2 cos(𝜃1 + 𝜃2) - x = 0,
% f2(𝜃1, 𝜃2) = l1 sin(𝜃1) + l2 sin(𝜃1 + 𝜃2) - y = 0.
function pos = getEFPosition(l1, l2, theta1, theta2)
    % evaluate the position of the end effector
    x = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
    pos = [x; y];
end

function J = getJacobian(l1, l2, theta1, theta2)
    % evaluate the Jacobian matrix
    % J11 = d f1 / d theta1 = -l1 sin(𝜃1) - l2 sin(𝜃1 + 𝜃2)
    J11 = -l1 * sin(theta1) - l2 * sin(theta1 + theta2);
    % J12 = d f1 / d theta2 = -l2 sin(𝜃1 + 𝜃2)
    J12 = -l2 * sin(theta1 + theta2);
    % J21 = d f2 / d theta1 = l1 cos(𝜃1) + l2 cos(𝜃1 + 𝜃2)
    J21 = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
    % J22 = d f2 / d theta2 = l2 cos(𝜃1 + 𝜃2)
    J22 = l2 * cos(theta1 + theta2);
    J = [J11, J12; J21, J22];
end
