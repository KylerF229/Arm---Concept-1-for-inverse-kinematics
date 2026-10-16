% Author: Kyler Fourthunders

clear;
clc;
close all;

% Arm dimensions
L1 = 0.40; % m
L2 = 0.30; % m

% Starting joint angles
theta1 = 132; % degrees
theta2 = -108;  % degrees

% Create figure
fig = uifigure('Name','Mechanical Arm');

ax = uiaxes(fig);
ax.Position = [50 120 500 400];

axis(ax,'equal')
grid(ax,'on')

xlim(ax,[-0.8 0.8])
ylim(ax,[-0.1 0.8])

xlabel(ax,'X Position (m)')
ylabel(ax,'Y Position (m)')
title(ax,'Mechanical Arm')

% Fixed the base
x0 = 0;
y0 = 0;

% Initial arm position
x1 = L1*cosd(theta1);
y1 = L1*sind(theta1);
x2 = x1 + L2*cosd(theta1 + theta2);
y2 = y1 + L2*sind(theta1 + theta2);

% Plot arm
armPlot = plot(ax,...
    [x0 x1 x2],...
    [y0 y1 y2],...
    '-o',...
    'LineWidth',2,...
    'MarkerSize',8);

% Slider for Joint 1
slider1 = uislider(fig,...
    'Position',[100 80 400 3],...
    'Limits',[-180 180],...
    'Value',theta1);

uilabel(fig,...
    'Position',[20 70 80 22],...
    'Text','Joint 1');

% Slider for Joint 2
slider2 = uislider(fig,...
    'Position',[100 40 400 3],...
    'Limits',[-180 180],...
    'Value',theta2);

uilabel(fig,...
    'Position',[20 30 80 22],...
    'Text','Joint 2');

% Update arm while sliders move
slider1.ValueChangingFcn = @(src,event) ...
    updateArm(event.Value,slider2.Value,L1,L2,armPlot);

slider2.ValueChangingFcn = @(src,event) ...
    updateArm(slider1.Value,event.Value,L1,L2,armPlot);


% Function
function updateArm(theta1,theta2,L1,L2,armPlot)

    % fixed Base
    x0 = 0;
    y0 = 0;

    % Joint 1
    x1 = L1*cosd(theta1);
    y1 = L1*sind(theta1);

    % End effector
    x2 = x1 + L2*cosd(theta1 + theta2);
    y2 = y1 + L2*sind(theta1 + theta2);

    % Update graph
    armPlot.XData = [x0 x1 x2];
    armPlot.YData = [y0 y1 y2];

end