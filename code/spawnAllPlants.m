function spawnAllPlants()
    plant = PlaceObject('HalfSizedRedGreenBrick.ply', location);
    verts = [get(plant,'Vertices'), ones(size(get(plant,'Vertices'),1),1)];
    set(plant,'Vertices',verts(:,1:3))


    Plant_Position = zeros(6,3);
    Plant_Position(1,:) =[0.12,0.6, 0.3];
    Plant_Position(2,:) =[0.02, 0.6, 0.3];
    Plant_Position(3,:) =[-0.1,0.6,0.3];
    Plant_Position(4,:) =[-0.2, 0.6, 0.3];
    Plant_Position(5,:) =[-0.3, 0.6, 0.3];
    Plant_Position(6,:) =[-0.4, 0.6, 0.3];
    Plant_Position(7,:) =[-0.5,0.6,0.6];
    Plant_Position(8,:) =[-0.6,0.6,0.6];
    Plant_Position(9,:) =[-0.7,0.6,0.6];
    
    %this section defines the wall positions of each brick
%     Table_Position = zeros(6,3);
%     Table_Position(1,:) =[0.25, 0.15, 0.5];
%     Table_Position(2,:) =[0.25, 0,0.5];
%     Table_Position(3,:) =[0.25,-0.15,0.5];
%     Table_Position(4,:) =[0.25,0.15,0.525];
%     Table_Position(5,:) =[0.25,0,0.525];
%     Table_Position(6,:) =[0.25,-0.15,0.525];
%     Table_Position(7,:) =[0.25,0.15,0.55];
%     Table_Position(8,:) =[0.25,0,0.55];
%     Table_Position(9,:) =[0.25,-0.15,0.55]; 
% 

    hold on;


    %% This section loads each brick into the environment while storing the object in a array.
    [f,v,data] = plyread('Plant.ply','tri');

    % Scale the colours to be 0-to-1 (they are originally 0-to-255
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    BrickVectCount = size(v,1);
    hold on;
    %place object function
    for i= 1:size(Plant_Position, 1)
        Plants(i) = trisurf(f,v(:,1)+ Plant_Position(i,1),v(:,2)+ Plant_Position(i,2), v(:,3) +Plant_Position(i,3)...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    end

    %activate dynamic lighting
    camlight;
end