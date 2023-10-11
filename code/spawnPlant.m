function spawnPlant(location)
    plant = PlaceObject('HalfSizedRedGreenBrick.ply', location);
    verts = [get(plant,'Vertices'), ones(size(get(plant,'Vertices'),1),1)];
    set(plant,'Vertices',verts(:,1:3))
