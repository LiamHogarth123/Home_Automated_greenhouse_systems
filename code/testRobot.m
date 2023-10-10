
L1 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi])
L2 = Link('d',0,'a',2,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180)
L4 = Link('d',0,'a',1,'alpha',-pi/2,'qlim',[-pi pi])
L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi])
L6 = Link('d',1,'a',0.5,'alpha',0,'qlim',[-pi pi])


robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot')                     % Generate the model
workspace = [-8 8 -8 8 -4 4];                                       % Set the size of the workspace when drawing the robot        
scale = 0.5;        
%q = zeros(1,6);                                                     % Create a vector of initial joint angles        
q = [0,pi/2,0,0,pi/2,0];
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

robot.teach(q);                              