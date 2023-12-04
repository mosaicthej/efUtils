
%% This code is available in eval3D.m
clear all;
clc;
clf;


ls=[0.8,0.7]';
theta0=[3*pi/4, pi/2, pi]; %Choose some random starting point.
n=10;
mode = 1;
%Start position
desired=[-0.2, -0.2, 0.5];
t=invKin3D(ls,theta0,desired,n,mode); 

           
% Set starting and ending position for cans
% Loads positions into matrices start_can_pos and end_can_pos
n_cans = 9;
load can_positions.mat;

% Counters for cans picked and placed
obj_picked = 0;
obj_placed = 0;
in_hand = 0;
deliver = 0;
steps = 6;
%%%%            Instructions            %%%%%%%%%%%%%%
%%% Keep in mind the following lines of codes
%%% Use 'clf; hold on;' at the beginning of your primary loop
%%% Once you pick an object plot it at the end effector using 
%%%       [pos_hand, J] = evalRobot3D(ls, t); 
%%%        plotCylinderWithCaps(0.1,pos_hand,0.2,12,[1 0 0],'z');
%%% Plot the complete scene using:
%%%        plot_scene(obj_picked, obj_placed, start_can_pos, end_can_pos, gca, ls, t);
%%%        drawnow();
%%% Make sure you update the obj_picked and obj_placed properly
% ... (rest of the eval3D.m script above)

%%%%%%% Enter your code here    %%%%%%%%%%%%%%%%%%%%%
for can_idx = 1:n_cans
    % Pick up can from the table
    for segment = 1:steps
        clf; hold on;
        desired = bezier(start_can_pos(can_idx, :), [0 0.3 0.2], steps, segment);
        t = invKin3D(ls, theta0, desired, n, mode);
        [pos_hand, J] = evalRobot3D(ls, t); 
        plotCylinderWithCaps(0.1, pos_hand, 0.2, 12, [1 0 0], 'z');
        plot_scene(obj_picked, obj_placed, start_can_pos, end_can_pos, gca, ls, t);
        drawnow();
    end
    obj_picked = obj_picked + 1;
    
    % Place can on the conveyor belt
    for segment = 1:steps
        clf; hold on;
        desired = bezier([0 0.3 0.2], end_can_pos(can_idx, :), steps, segment);
        t = invKin3D(ls, theta0, desired, n, mode);
        [pos_hand, J] = evalRobot3D(ls, t); 
        if segment == steps % Only plot can on end effector at the last segment
            plotCylinderWithCaps(0.1, pos_hand, 0.2, 12, [1 0 0], 'z');
        end
        plot_scene(obj_picked, obj_placed, start_can_pos, end_can_pos, gca, ls, t);
        drawnow();
    end
    obj_placed = obj_placed + 1;
end

% Provide explanation for what happens if Newton's method stops converging
% and potential strategies for dealing with targets far from the end effector
% in the comments within the code.
%
% For example:
% If Newton's method fails to converge, this could be due to the target point being
% outside the manipulator's reachable workspace or near a singularity where small
% changes in joint angles result in large end effector movements. One strategy to
% handle this issue is to implement a fallback method, such as moving the target
% point closer to the workspace boundary or using a different set of joint angles
% as the initial guess. Additionally, path planning with intermediate waypoints
% can help navigate around singularities and unreachable areas.

%%%%%%%%%~~~~~END~~~~~~%%%%%%%%%%%%%%%%%%%
        %When moving from table to converyor, plot can at the end of arm


function desired = bezier(start_pos, end_pos, steps, curr_segment)
    %The trajectory smoothly varies between:
    %   1. start point
    %   2. start point + vertical offset
    %   3. end point + vertical offset
    %   4. end point
    arr_steps = linspace(0, 1, steps);
    tb = arr_steps(curr_segment);
    start_ver = start_pos;
    start_ver(3) = 1.5;
    end_ver = end_pos;
    end_ver(3) = 1.5;
    bez1 = (1-tb)*((1-tb)*start_pos+tb*start_ver) + tb*((1-tb)*start_ver + tb*end_ver);
    bez2 = (1-tb)*((1-tb)*start_ver + tb*end_ver) + tb*((1-tb)*end_ver + tb*end_pos);
    desired = (1-tb)*bez1 + tb*bez2;
end

function plot_scene(obj_picked, obj_placed, start_can_pos, end_can_pos, gca, ls, t)
%% Plotting the complete scene in every iteration of the inverse kinematics
 %%Input Arguments --- 
 %%%  objects_picked - no of objects already picked
 %%%  obj_placed - no of objects placed
 %%%  start_can_pos - starting posing of cans on table
 %%%  end_can_pos   - ending posing of cans on conveyor
 %%%  gca - current figure parameter
 %%%  ls - link lengths constant [0.8 0.7]
 %%%  t - current joint angles
 %%%%%%~~~~~~~~~~~END~~~~~~~~~~~%%%%%%%
 %%% Output- plots the current scene 
% Plot: cans, static objects (floor, table, conveyor, etc) and robot
    hold on;
    daspect([1 1 1])
    plot_cans(obj_picked, obj_placed, start_can_pos, end_can_pos);
    plot_static(gca);
    plotRobot3D(ls,t);
    hold off;
    
end

function plot_cans(obj_picked, obj_placed, start_can_pos, end_can_pos)
 %% Input Arguments --- 
 %%%  objects_picked - no of objects already picked
 %%%  obj_placed - no of objects placed
 %%%  start_can_pos - starting posing of cans on table
 %%%  end_can_pos   - ending posing of cans on conveyor
 %%%%%%~~~~~~~~~~~END~~~~~~~~~~~%%%%%%%
 %%% Output- plots the current cans on table and conveyor 
    % Params for drawing cans
    nSides = 12;           % number of "sides" of the cyl
    color = [1 0 0];        % color of each cyl
    r = 1/10; 
    height = 0.2;

    n_objs = size(start_can_pos, 1);
    for i = 1:n_objs
        if i > obj_picked
            plotCylinderWithCaps(r,start_can_pos(i,:),height,nSides,color,'z');
        end
        if i <= obj_placed
            plotCylinderWithCaps(r, end_can_pos(i,:), height, nSides, color, 'z');
        end
    end
end

function plot_static(gca)
    %% This routine plots the complete scene with table, conveyer and cans.
    %Plot cuboids
    brown = '#964B00';
    grey = '#808080';
    black = '#000000';
    draw_cuboid(gca,1,1,0.2,[-0.1, -1.3, -0.4],brown) % table
    draw_cuboid(gca,4,4,0.2,[-2, -2, -1],grey) % floor
    draw_cuboid(gca,1,2,0.2,[-1.3, -1.5, 0],black) % conveyor belt
    plotCylinderWithCaps(0.1,[0.1, -1.5, -1.3],1,12,[0 0 0],'x'); % end of belt
    % rollers
    dy = 0.3;
    for i = 0:6
        plotCylinderWithCaps(0.09,[0.1, -1.49+i*dy, -1.32],1.04, 12,[0.9 0.9 0.9],'x');
    end
end

function draw_cuboid(ax, L,W,H, offset, color)
 %% Input Arguments --- 
 %%%  ax - current figure parameter
 %%%  L - Length of cuboid
 %%%  W - Width of cuboid
 %%%  H - Height of cuboid
 %%%  offset - coordinates of center of cuboid
 %%%  color - color code to fill the cuboid
 %%%%%%~~~~~~~~~~~END~~~~~~~~~~~%%%%%%%
 %%% Output- plots a cuboid at the offset location
    vert = [0 0 0;L 0 0;L W 0;0 W 0;0 0 H;L 0 H;L W H;0 W H];
    vert(:,1) = vert(:,1) + offset(1);
    vert(:,2) = vert(:,2) + offset(2);
    vert(:,3) = vert(:,3) + offset(3);
    
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
    patch(ax, 'Vertices',vert,'Faces',fac, 'FaceColor',color)
end

function [h1, h2, h3] = plotCylinderWithCaps(r,cnt,height,nSides,color,orientation)
 %% Input Arguments --- 
 %%%  r- radius of cylinder 
 %%%  cnt - center of the cylinder base
 %%% height - height of the cylinder
 %%%  nSides- no of sides of the cylinder
 %%%  color- color of sides
 %%%  orientation- current orientation of the cylinder (horizontal('x') or vertical)
 %%%%%%~~~~~~~~~~~END~~~~~~~~~~~%%%%%%%
 %%% Output- Use this (h1,h2,h3) to plot the surface of the cylinder 
    [X,Y,Z] = cylinder(r,nSides);
    X = X + cnt(1); 
    Y = Y + cnt(2); 
    Z = Z * height + cnt(3); 
    if orientation == 'x'
        h1 = surf(Z,Y,X,'facecolor',color,'LineStyle','none');
        h2 = fill3(Z(1,:),Y(1,:),X(1,:),color);
        h3 = fill3(Z(2,:),Y(2,:),X(2,:),color);
    else
        h1 = surf(X,Y,Z,'facecolor',color,'LineStyle','none');
        h2 = fill3(X(1,:),Y(1,:),Z(1,:),color);
        h3 = fill3(X(2,:),Y(2,:),Z(2,:),color);
    end
end
