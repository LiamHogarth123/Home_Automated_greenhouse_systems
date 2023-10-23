function spawnEnvironment()

workspace = [-5 5 -5 5 -5 3]; % creating the workspace

%% Creating the environment
    % Concrete floor
    surf([-4,-4;3,3], ...
         [-4,3;-4,3], ...
         [0,0;0,0], ...
         'CData',imread('concrete.jpg'), ...
         'FaceColor','texturemap');
    hold on;  % To keep adding more assets to the environment
    
    % Brick Walls
    surf([-4, -4; 3, 3], ...
         [3, 3; 3, 3], ...
         [0, 2; 0, 2], ...
         'CData',imrotate(imread('brick_wall.jpg'), 90), ...  % Rotate the texture 90 degrees
         'FaceColor','texturemap');
    surf([3, 3; 3, 3], ...
         [-4, -4; 3, 3], ...
         [0, 2; 0, 2], ...
         'CData',imrotate(imread('brick_wall.jpg'), 90), ...  % Rotate the texture 90 degrees
         'FaceColor','texturemap');
    


    