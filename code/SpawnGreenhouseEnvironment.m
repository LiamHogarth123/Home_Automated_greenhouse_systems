function SpawnGreenhouseEnvironment()
close all
clc
clf
clear all
%% Lab3Starter Create a Puma 560 DH SerialLink Model

%% defining and inislisting robot

%defining the main properities


hold on;

% creating concrete floor
surf([-2.5,-2.5;2,2] ...
,[-3,2;-3,2] ...
,[0.01,0.01;0.01,0.01] ...
,'CData',imread('concrete.jpg') ...
,'FaceColor','texturemap');

view(3)


house = PlaceObject('GreenHouseLargerNowall.ply', [0,0,0]);
xlim([-2.5, 2])
ylim([-3, 2])
zlim([0, 2])

% barriers
f1 = PlaceObject('barrier1.5x0.2x1m.ply',[0.65,-1.85,0]);
verts = [get(f1,'Vertices'), ones(size(get(f1,'Vertices'),1),1)]; 
verts(:, 1) = verts(:, 1) * 1.15;
set(f1,'Vertices',verts(:,1:3));

f2 = PlaceObject('barrier1.5x0.2x1m.ply',[-0.85,-1.85,0]);
verts = [get(f2,'Vertices'), ones(size(get(f2,'Vertices'),1),1)]; 
verts(:, 1) = verts(:, 1) * 1.15;
set(f2,'Vertices',verts(:,1:3));

f3 = PlaceObject('barrier1.5x0.2x1m.ply',[0.75,-1.85,0]);
verts = [get(f3,'Vertices'), ones(size(get(f3,'Vertices'),1),1)] * trotz(pi/2);
verts(:, 2) = verts(:, 2) * 1.23;
set(f3,'Vertices',verts(:,1:3));

% Fire extinguisher
fire = PlaceObject('fireExtinguisherElevated.ply',[1.48,-2,0.53]);

% desk
table2 = PlaceObject('tableBrown2.1x1.4x0.5m.ply', [1, -4.5, -0.1]);
verts = [get(table2, 'Vertices'), ones(size(get(table2, 'Vertices'), 1), 1)];
verts(:, 1) = verts(:, 1) * 0.6; 
verts(:,2) = verts(:,2) * 0.5;
verts(:, 3) = verts(:, 3) * 2;
set(table2, 'Vertices', verts(:, 1:3));

% laptop
laptop = PlaceObject('laptop.ply', [0.95, -2.1, 0.82]);

% Emergency stop button
button = PlaceObject('emergencyStopButton.ply',[0.55,-2.1,0.8]);

% first aid
aid = PlaceObject('firstAid.ply',[2.25,-0.08,0.65]);
verts = [get(aid,'Vertices'), ones(size(get(aid,'Vertices'),1),1)] * trotz(pi/2);
set(aid,'Vertices',verts(:,1:3));


camlight;