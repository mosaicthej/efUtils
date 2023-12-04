% Q2.1
% a.
%l = [0.8;0.7];
% theta = [any;pi/2;any];
%theta = [0;pi/2;0];
%figure
%plotRobot3D(l,theta)
% Now the elbow is at pi/2
%
% b.
% theta = [pi/2;0;any];
%theta = [pi/2;0;0];
%figure
%plotRobot3D(l,theta)

% Link lengths
l = [0.8; 0.7];

% Configuration A: Elbow at pi/2
% thetaA = [any;pi/2;any];
thetaA = [0, pi/2, 0];
figure
plotRobot3D(l,thetaA)
[posA, JA] = evalRobot3D(l, thetaA);

condJA = cond(JA); % Condition number of Jacobian in Configuration A
 plotRobot3D(l, thetaA); % Uncomment to visualize

% Configuration B: Arm Fully Stretched Upward
% thetaB = [pi/2;0;any];
thetaB = [pi/2, 0, 0];
figure
plotRobot3D(l,thetaB)
[posB, JB] = evalRobot3D(l, thetaB);
condJB = cond(JB); % Condition number of Jacobian in Configuration B
 plotRobot3D(l, thetaB); % Uncomment to visualize

% Display condition numbers as comments
fprintf('Condition number for Configuration A: %f\n', condJA);
fprintf('Condition number for Configuration B: %f\n', condJB);
% Condition number for Configuration A is 2.491491
% Condition number for Configuration B is Inf
% For Configuration A, thetaA = [any;pi/2;any];
% For Configuration B, thetaB = [pi/2;0;any];
% Configuration B gives a singular Jacobian matrix, since the condition number is Inf
% if thetaC = [pi/2, 0, pi/2], Configuration B also gives a singular Jacobian matrix
thetaC = [pi/2, 0, pi/2];
[posC, JC] = evalRobot3D(l, thetaC);
condJC = cond(JC);
fprintf('Condition number for Configuration B with diff rotational-angle: %f\n', condJC);

