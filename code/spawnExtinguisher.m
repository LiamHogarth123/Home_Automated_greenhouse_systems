function [vertex,faces,faceNormals] = spawnExtinguisher

    fire = PlaceObject('fire.ply',[0.6,-0.7,0.5]);
    verts = [get(fire,'Vertices'), ones(size(get(fire,'Vertices'),1),1)];
    set(fire,'Vertices',verts(:,1:3));
    vertex = verts(:, 1:3);
    faces = get(fire, 'Faces');
    
    % Calculate face normals
    faceNormals = zeros(size(faces, 1), 3);
    
    for i = 1:size(faces, 1)
        % Get the vertices of the current face
        faceVertices = vertex(faces(i, :), :);
    
        % Calculate the normal vector for the face
        normalVector = cross(faceVertices(2, :) - faceVertices(1, :), faceVertices(3, :) - faceVertices(1, :));
    
        % Normalize the normal vector to have a unit length
        normalVector = normalVector / norm(normalVector);
    
        % Store the normal vector in the array
        faceNormals(i, :) = normalVector;
    end
end