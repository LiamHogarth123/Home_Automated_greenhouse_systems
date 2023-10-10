%% Main function that will run the code - this will eventually be implemented within the GUI

% spawn the static environment
spawnEnvironment()

q = [0 0 0 0 0 0 0];
r = LinearUR5();
r.model;
r.model.teach(q)