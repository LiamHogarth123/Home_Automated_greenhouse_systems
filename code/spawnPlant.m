function spawnPlant(location)
    plant = PlaceObject('HalfSizedRedGreenBrick.ply', location);
    verts = [get(placeBrick,'Vertices'), ones(size(get(placeBrick,'Vertices'),1),1)];
    set(plant,'Vertices',verts(:,1:3))
