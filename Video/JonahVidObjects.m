%% Gantry
clc
clear all
close all

LinearUR5
axis([-1.5 1.5 -1.5 1.5 -0.01 1.6])
view(-27,19);

hold on
railsObj1 = PlaceObject('LinearUR5Link0.ply',[0,0,0]);
verts = [get(railsObj1,'Vertices'), ones(size(get(railsObj1,'Vertices'),1),1)] * trotz(pi/2) * troty(pi/2) * trotz(pi);
verts(:,1) = verts(:,1) + 1;
set(railsObj1,'Vertices',verts(:,1:3))

railsObj2 = PlaceObject('LinearUR5Link0.ply',[0,0,0]);
verts = [get(railsObj2,'Vertices'), ones(size(get(railsObj2,'Vertices'),1),1)] * trotz(-pi/2);
verts(:,1) = verts(:,1) + 1.25;
verts(:,3) = verts(:,3) + 0.8;
verts(:,3) = verts(:,3) * 1.5;
set(railsObj2,'Vertices',verts(:,1:3))

railsObj3 = PlaceObject('LinearUR5Link0.ply',[0,0,0]);
verts = [get(railsObj3,'Vertices'), ones(size(get(railsObj3,'Vertices'),1),1)] * trotz(pi/2);
verts(:,1) = verts(:,1) - 0.95;
verts(:,3) = verts(:,3) + 0.8;
verts(:,3) = verts(:,3) * 1.5;
set(railsObj3,'Vertices',verts(:,1:3))

railsLink1 = PlaceObject('LinearUR5Link1.ply',[0,0,0]);
verts = [get(railsLink1,'Vertices'), ones(size(get(railsLink1,'Vertices'),1),1)] * trotz(pi/2) * trotx(pi);
verts(:,1) = verts(:,1) - 0.8;
verts(:,3) = verts(:,3) + 0.05;
set(railsLink1,'Vertices',verts(:,1:3))

railsLink2 = PlaceObject('LinearUR5Link1.ply',[0,0,0]);
verts = [get(railsLink2,'Vertices'), ones(size(get(railsLink2,'Vertices'),1),1)] * trotz(pi/2) * trotx(pi);
verts(:,1) = verts(:,1) + 1.1;
verts(:,3) = verts(:,3) + 0.05;
set(railsLink2,'Vertices',verts(:,1:3))

railsLink3 = PlaceObject('LinearUR5Link1.ply',[0,0,0]);
verts = [get(railsLink3,'Vertices'), ones(size(get(railsLink3,'Vertices'),1),1)] * trotx(pi/2) * trotz(pi/2);
verts(:,1) = verts(:,1) - 1;
verts(:,3) = verts(:,3) + 1.2;
set(railsLink3,'Vertices',verts(:,1:3))

railsLink4 = PlaceObject('LinearUR5Link1.ply',[0,0,0]);
verts = [get(railsLink4,'Vertices'), ones(size(get(railsLink4,'Vertices'),1),1)] * trotx(pi/2) * trotz(-pi/2);
verts(:,1) = verts(:,1) + 1.3;
verts(:,3) = verts(:,3) + 1.2;
set(railsLink4,'Vertices',verts(:,1:3))

%% plant
% clc
% close all
% clear all
pause
axis([-0.5 0.5 0.5 1.5 -0.01 0.5])
% view(-180,5);
view(210,16);

% camlight
% hold on

plantObj1 = PlaceObject('Plant.ply',[0,1,0]); % Placing plants into world
verts = [get(plantObj1,'Vertices'), ones(size(get(plantObj1,'Vertices'),1),1)];
% verts(:,1) = verts(:,1) + 1.3;
set(plantObj1,'Vertices',verts(:,1:3));

pause(1)

sensor = PlaceObject('soil-moisture-sensor-probe.ply',[-0.05,1,0.5]); % Placing plants into world
verts = [get(sensor,'Vertices'), ones(size(get(sensor,'Vertices'),1),1)];
% verts(:,3) = verts(:,3) + 0.2;
set(sensor,'Vertices',verts(:,1:3));

pause(1)

for i=1:100
    updatedSensorVerts = [verts,ones(size(verts,1),1)];% * transl([0,0,i*0.005]);
    updatedSensorVerts(:,3) = updatedSensorVerts(:,3) - i*(0.0036/1);
    set(sensor,'Vertices',updatedSensorVerts(:,1:3));
    drawnow();
    % pause(0.01)
end

