function animateRobot(qPath, robot)%, movingBrick)
    %flyingBrick = [];
    for i=1:size(qPath)
        %disp(['Step ',num2str(i)]); % Displays number of steps
        %disp(qPath(i,:)); % Prints joint values
        robot.model.animate(qPath(i,:));
        % if movingBrick == true
        %     tr = robot.model.fkine(robot.model.getpos).T;
        %     updatedBrickVerts = [brickVertices,ones(size(brickVertices,1),1)] * tr';
        %     set(brickObjects{count},'Vertices',updatedBrickVerts(:,1:3));
        % end
        
        drawnow();
        % Delete and spawn new brick at robot end effector
        % if movingBrick == true
        %     T = robot.model.fkine(qPath(i,:)); % transform the joint angles to coordinates
        %     pos = transl(T); % global coordinates
        %     %orientation = tr2eul(Tr)
        %     %R = eul2rotm(orientation, "ZYX");
        %     try 
        %         delete(flyingBrick); % deletes moving brick if it has been picked up already
        %     catch
        %     end
        %     flyingBrick = PlaceObject('HalfSizedRedGreenBrick.ply',pos); % creates the brick at end effector location
        %     verts = [get(flyingBrick,'Vertices'), ones(size(get(flyingBrick,'Vertices'),1),1)];
        %     set(flyingBrick,'Vertices',verts(:,1:3));
        % end
        % pause(0);
    end
    % delete(flyingBrick); % deletes final brick before finishing function, end effector position
    disp(' ');
    disp(['Current joint values: [', num2str(qPath(i,:)), ']']); % Prints joint values after finishing motion
end

