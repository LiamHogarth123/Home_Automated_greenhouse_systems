% Create an instance of your LinearUR5 robot
robot = A0509();
q = [0 0 0 0 0 0; 0 0 0 0 0 0];
hold on
% aid = PlaceObject('firstAid.ply',[0,0.05,0.5]);
% verts = [get(aid,'Vertices'), ones(size(get(aid,'Vertices'),1),1)];
% set(aid,'Vertices',verts(:,1:3));
% vertex = verts(:, 1:3);
% faces = get(aid, 'Faces');
% 
% % Calculate face normals
% faceNormals = zeros(size(faces, 1), 3);
% 
% for i = 1:size(faces, 1)
%     % Get the vertices of the current face
%     faceVertices = vertex(faces(i, :), :);
% 
%     % Calculate the normal vector for the face
%     normalVector = cross(faceVertices(2, :) - faceVertices(1, :), faceVertices(3, :) - faceVertices(1, :));
% 
%     % Normalize the normal vector to have a unit length
%     normalVector = normalVector / norm(normalVector);
% 
%     % Store the normal vector in the array
%     faceNormals(i, :) = normalVector;
% end

[vertex,faces,faceNormals] = spawnExtinguisher;

% centerpnt = [0.5,0,1];
% side = 0.5;
% plotOptions.plotFaces = true;
% [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

checkCollision(robot.model,q(1,:),faces,vertex,faceNormals,false)