clear all
close all
clc
%%
L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot') % Generate the model
workspace = [-1 1 -1 1 -0.05 1]; % Set the size of the workspace when drawing the robot        
scale = 0.5;
%q = [0,pi/2,0,0,pi/2,0];
q = zeros(1,6); % Create a vector of initial joint angles
figure(1)
robot.plot(q,'workspace',workspace,'scale',scale); % Plot the robot
% robot.teach(q);

%%
qVals = [   0,-pi/2,0,-pi/2,0,0;
            0,0,0,0,0,0;
            0,0,0,0,0,0;
            0,0,0,0,0,0
        ];

% q0 = [0,-pi/2,0,-pi/2,0,0];
q0 = [0,0,0,0,0,0];

% q = robot.ikcon(q0, transl([]));
% q = robot.ikcon(transl([0.2,0.2,0.2]),qVals(1,:));
q1 = transl([0.2,0.2,0.2])
tr = robot.fkine(q0).T
pos = transl(robot.fkine(q0))

%%
q0 = [0,0,0,0,0,0];
qVals = [   deg2rad([23.5 -18 -5.5 1 0 23]);
            deg2rad([23.5 -18 -5.5 -23.5 0 0]);
            deg2rad([23.5 -18 -5.5 -23.5 0 46.7]);
            %
            deg2rad([80 -26.5 19 -42 0 50]);
            deg2rad([80 -26.5 19 -42 0 0]);
            deg2rad([80 -26.5 19 -42 0 50]);
            deg2rad([0 -26.5 19 -42 0 50]);
            % deg2rad([59 -58 38 -33 -2 54]);
            % deg2rad([0 -45 42 -52 0 54]);
            q0
        ];



for i=1:size(qVals)
    qPath = jtraj(robot.getpos,qVals(i,:),25);
    for j=1:size(qPath)
        robot.animate(qPath(j,:));
        drawnow();
        % disp(' ');
        % disp(['Current joint values: [', num2str(qPath(j,:)), ']']); % Prints joint values after finishing motion
    end
    % animateRobot(qPath, robot);
    % disp(['loop: ',num2str(i)]);
end
