function SpawnGreenhouseEnvironment()
close all
clc
clf
clear all
%% Lab3Starter Create a Puma 560 DH SerialLink Model

%% defining and inislisting robot

%defining the main properities

robot = LinearUR5;

hold on;

% creating concrete floor
surf([-1.8,-1.8;1.8,1.8] ...
,[-1.8,1.8;-1.8,1.8] ...
,[0.01,0.01;0.01,0.01] ...
,'CData',imread('concrete.jpg') ...
,'FaceColor','texturemap');

view(3)


house = PlaceObject('GreenHouseLargerNowall.ply', [0,0,0]);
xlim([-2, 2])
ylim([-2, 2])
zlim([0, 2])