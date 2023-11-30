% Code 2-9 | Broyden Method
clc; clear
fo = @(x) [x(1)^3+(x(1)^2)*x(2)-x(1)*x(3)+6;    %function
    exp(x(1))+exp(x(2))-x(3);
    x(2)^2-2*x(1)*x(3)-4];
Xo = [-1; -2; 1];                               %satrt points

% try use:
% f1(𝜃1, 𝜃2) = l1 cos(𝜃1) + l2 cos(𝜃1 + 𝜃2) - x = 0,
% f2(𝜃1, 𝜃2) = l1 sin(𝜃1) + l2 sin(𝜃1 + 𝜃2) - y = 0.
g = @(l, theta) [l(1)*cos(theta(1)) + l(2)*cos(theta(1) + theta(2));
    l(1)*sin(theta(1)) + l(2)*sin(theta(1) + theta(2))];

theta_0 = [0.1; 0.1];
P = [0.7071; 1.7071]; l = [1; 1];
X = theta_0 % initial guess and param
% if g = [x; y], then
% f = [g(1) - x; g(2) - y];
% which meausres the residual based on the current guess of theta,
% and it is the subject for optimization (minimize to zero);
f = @(theta) [g(l, theta)(1) - P(1); g(l, theta)(2) - P(2)];
% f = @(pos, l, theta) [l(1)*cos(theta(1)) + l(2)*cos(theta(1) + theta(2)) - pos(1);

error = 1e-3;
itr = 100;
%Method_________________________________________________________________
n = numel(X);
Fx0 = f(X);
Jac = zeros(n); h = 1e-5;
for j = 1:n                                    %Jacobian
    Xh = X; Xh(j) = Xh(j)+h;
    Jac(:,j) = (f(Xh)-Fx0)/h;
end
for i = 1:itr
    Tab(i,:) = [i-1,X.',Fx0.'];           %for illustration
    if norm(Fx0) <= error, break; end
    dX = -Jac\Fx0;
    X = X+dX;
    Fx = f(X);
    % Jac = Jac+(Fx-Fx0-Jac*dX)/dot(dX,dX)*dX.'; %Jacobian Broyden
    % instead, try with my solution to see if it works
    %	B = B + ((y - B*delta_theta)* delta_theta') / (delta_theta' * delta_theta);
    Jac = Jac+((Fx-Fx0-Jac*dX)* dX')/(dX'*dX);
    Fx0 = Fx;
end
%Illustration___________________________________________________________
div = ['\n---' repmat('------------',[1,2*n]), '\n'];
fprintf('itr');
fprintf('          x%d',1:n);
fprintf('       f%d(X)',1:n);
fprintf(div);
fprintf(['%3.0f',repmat('%12.5g',[1,2*n]),'\n'],Tab');
fprintf(['\b' div]);
fprintf('X = ');fprintf('[%12.5g ]\n    ',X.');fprintf('\b\b\b\b');
