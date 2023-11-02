%% Main function that will run the code - this will eventually be implemented within the GUI

% spawn the static environment
% spawnEnvironment()
% 
% q = [0 0 100 0 0 0 0];
% r = LinearUR5();
% r.model;
% 
% 
% % r.model.teach(q)
% r.model.animate(q);
% drawnow();
% x=r.model.getpos
% y=transl(r.model.fkine(x))
% r.model.ikcon(transl(y))

% spawnLinearUR5();

% spawnTest(q);


% myRobot = SpawnLinearUR5();
% % myRobot.spawnLinearUR5();
% myRobot.moveRobot(q);

% classdef main %< GUI
%     methods
%         function self = main()
%             x=0;
%             while (true)
%                 x = x+1
%                 getEstopStatus
%                 pause(1)
%             end
%         end
%     end
% end

% Create an instance of your LinearUR5 robot
robot = LinearUR5();
robot.model;

% Define the joint angles (modify these values according to your robot's configuration)
q = [0, 0, 0, 0, 0, 0, 0];

% Calculate the end-effector transformation matrix (TCP)
TCP_transform = robot.model.fkine(q);

% Extract XYZ position from the transformation matrix
TCP_position = TCP_transform(1:3, 4);

% Display the XYZ position of the end-effector
disp('End-Effector Position (XYZ):');
disp(TCP_position);

% Calculate and display the XYZ positions of each joint
joint_positions = zeros(6, 3);  % Create a matrix to store joint positions

for i = 1:6
    q_temp = q;
    q_temp(i) = 0;  % Set the specific joint to 0 while keeping others constant
    TCP_transform = robot.model.fkine(q_temp);
    joint_positions(i, :) = TCP_transform(1:3, 4);
end

disp('XYZ Positions of Each Joint:');
disp(joint_positions);