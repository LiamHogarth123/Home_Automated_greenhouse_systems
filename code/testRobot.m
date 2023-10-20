clear all
clf
clc

%% 6 link A0509
L1 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi],'offset',pi/2)
L3 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',pi/2)
L4 = Link('d',1,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',pi)
L6 = Link('d',0.5,'a',0,'alpha',0,'qlim',[-pi pi],'offset',0)

% L1 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
% L2 = Link('d',0,'a',0,'alpha',0,'qlim',[-pi pi],'offset',pi/2)
% L3 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',pi/2)
% L4 = Link('d',1,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
% L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',pi)
% L6 = Link('d',0.5,'a',0,'alpha',0,'qlim',[-pi pi],'offset',0)

% L1 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
% L2 = Link('d',0.5,'a',1,'alpha',0,'qlim',[-pi pi],'offset',pi/2)
% L3 = Link('d',-0.5,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',pi/2)
% L4 = Link('d',1,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
% L5 = Link('d',0.5,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',pi)
% L6 = Link('d',0.5,'a',0,'alpha',0,'qlim',[-pi pi],'offset',0)

% L1 = Link('d',0,'a',1,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
% L2 = Link('d',0,'a',2,'alpha',0,'qlim',[-pi pi],'offset',pi/2)
% L3 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',0)
% L4 = Link('d',0,'a',1,'alpha',-pi/2,'qlim',[-pi pi],'offset',0)
% L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',pi/2)
% L6 = Link('d',1,'a',0.5,'alpha',0,'qlim',[-pi pi],'offset',0)



robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot')            % Generate the model
workspace = [-3 3 -3 3 -1 3];                                       % Set the size of the workspace when drawing the robot        
scale = 0.5;                                                    % Create a vector of initial joint angles        
%q = [0,pi/2,0,0,pi/2,0];
q = zeros(1,6);
figure(1)
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
robot.teach(q);       


% UR3L1 = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
% UR3L2 = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
% UR3L3 = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
% UR3L4 = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
% UR3L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
% UR3L6 = Link('d',	0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
% 
% workspace = [-1 1 -1 1 -1 1]; 
% UR3robot = SerialLink([UR3L1 UR3L2 UR3L3 UR3L4 UR3L5 UR3L6],'name','myRobotUR3')
% figure(2)
% UR3robot.plot(q,'workspace',workspace,'scale',scale)
% UR3robot.teach(q);
