%This code is available in eval2D.m
% make sure you define n and mode, the ones
% below are for illustration
n = 10;
mode = 0; % 1 for Newton, 0 for Broyden
ms = "broyden";
if mode==1
	ms = "newton";
end
disp("mode is ");
disp(ms);

ls=[0.5,0.5]';
t=rand(2,1); %Choose some random starting point.

clf;
plotRobot2D(ls,t);
hold off;

while(1)
  desired=ginput(1)'; %Get desired position from user

  clf;
  plot(desired(1),desired(2),'*');
  hold on;
  plotRobot2D(ls,t,':');
  
  %Solve and display the position
  t=invKin2D(ls,t,desired,n,mode); 
  plotRobot2D(ls,t); 
  hold off;
end
