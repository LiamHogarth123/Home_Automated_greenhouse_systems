%% Main function that will run the code - this will eventually be implemented within the GUI

% spawn the static environment
spawnEnvironment()

q = [0 0 100 0 0 0 0];
r = LinearUR5();
r.model;


% r.model.teach(q)
r.model.animate(q);
drawnow();
x=r.model.getpos
y=transl(r.model.fkine(x))
r.model.ikcon(transl(y))

% spawnLinearUR5();

% spawnTest(q);


% myRobot = SpawnLinearUR5();
% % myRobot.spawnLinearUR5();
% myRobot.moveRobot(q);