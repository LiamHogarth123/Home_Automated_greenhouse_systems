clear all
clf
clc

%% 6 link A0509
L1 = Link('d',0.155,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
L2 = Link('d',0,'a',0.409,'alpha',0,'qlim',[-pi pi],'offset',pi/2)
L3 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',pi/2)
L4 = Link('d',0.367,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',pi)
L6 = Link('d',0.127,'a',0,'alpha',0,'qlim',[-pi pi],'offset',0)

% L1 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
% L2 = Link('d',0,'a',0,'alpha',0,'qlim',[-pi pi],'offset',pi/2)
% L3 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',pi/2)
% L4 = Link('d',1,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
% L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',pi)
% L6 = Link('d',0.5,'a',0,'alpha',0,'qlim',[-pi pi],'offset',0)

robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot') % Generate the model
workspace = [-1 1 -1 1 -0.05 1]; % Set the size of the workspace when drawing the robot        
scale = 0.5;
%q = [0,pi/2,0,0,pi/2,0];
q = zeros(1,6); % Create a vector of initial joint angles
figure(1)
robot.plot(q,'workspace',workspace,'scale',scale); % Plot the robot
%robot.teach(q);
% A0509

%% A0509 test movement
aR = A0509;

plant1 = [-0.9,   -0.4,   0];
plant2 = [0.9, 0.5,   0];
plant3 = [-0.85, 0.4,   0];
plant4 = [-0.4, -0.5,   0];
plant5 = [-0.2, -0.4,   0];
plant6 = [0,    -0.5,   0];
plant7 = [0.45, 0.05,   0];
plant8 = [0.3,  0.05,   0];
plant9 = [0.4,  0.4,    0];
plants = [plant1;plant2;plant3;plant4;plant5;plant6;plant7;plant8;plant9];
plants = [plant1;plant2;plant3];

plantPoses = zeros(3,6);
%shelfPoses = zeros(9,6);
for i=1:size(plants)
    plantPoses(i,:) = aR.model.ikcon(transl(plants(i,:))*trotx(pi));
    %shelfPoses(i,:) = r.model.ikcon(transl(e.endBricks(i,:))*trotx(pi));
end

axis([-1.3,1,-1,1,0,1.5])

for i=1:size(plantPoses)
    qPath = jtraj(aR.model.getpos,plantPoses(i,:),100); % Creates path of robot current pos to brick at index i
    animateRobot(qPath,aR); % Steps over the qPath and animates the robot
    pos = transl(aR.model.fkine(aR.model.getpos))
    %fkine = aR.model.fkine(aR.model.getpos);
    %translate = transl(fkine)
    pause(0.5)
end
animateRobot(jtraj(aR.model.getpos,aR.model.ikcon(transl(0,0,0.8)*trotx(pi)),100),aR)
pause(1)
for i=1:size(plantPoses)
    qPath = jtraj(robot.getpos,plantPoses(i,:),50); % Creates path of robot current pos to brick at index i
    for i=1:size(qPath)
        robot.animate(qPath(i,:));
        drawnow();
    end
    pos = transl(robot.fkine(robot.getpos))
    %fkine = aR.model.fkine(aR.model.getpos);
    %translate = transl(fkine)
    pause(0.5)
end