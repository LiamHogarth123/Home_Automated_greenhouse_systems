classdef Lab2ClassTest <handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        robot1 %object for the LinearUR3 so i can call it in other functions
        UR5Grip %object for the Gripper so i can call it in other functions
        robot2 %object for the LinearUR3 so i can call it in other functions
        A0509Grip %object for the Gripper so i can call it in other functions
        Plants
        i
        Table_Position
        Plant_Position
    end

    methods
    
        function self = Lab2ClassTest()
            clf;
            hold on;
            SpawnGreenhouseEnvironment();
            self.robot1 = LinearUR5;
            self.robot1.model;
            self.UR5Grip = UR5_Gripper;
            self.robot2 = A0509;
            
            drawnow()
%             self.A0509Grip = A0509_Gripper;
            self.Populate_variables(self)
            
            self.placePlants(self);
            self.PlacePlantOnTable(self);
           

        end

    end

    methods(Static)
        
        function placePlants(self)
             %these brick are in a lined up configuration 
            
            [f,v,data] = plyread('Plant.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            BrickVectCount = size(v,1);
            hold on;
            %place object function
            
            for i= 1:size(self.Plant_Position, 1)
                self.Plants(i) = trisurf(f,v(:,1)+ self.Plant_Position(i,1),v(:,2)+ self.Plant_Position(i,2), v(:,3) +self.Plant_Position(i,3)...
                    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end
            camlight;
            hold on;
        end

   


        function PlacePlantOnTable(self)
            %% placing bricks on table
            
            offset = 0;
            steps = 50;
    


            
            for i= 1:size(self.Plant_Position, 1)
                %%get brick -----------------------------------------------------

                %stores the current brick's position while adding a offset for the
                Current_Plant = self.Plant_Position(i,:);
                Current_Plant(3) = Current_Plant(3) + offset;
        
                %reads the current position
                current_position = self.robot1.model.getpos();
                
                %calculates the required translation
                requiredTranslation = transl(Current_Plant) * troty(90,'deg')* trotx(90,'deg');
        
                %caclutes the required q angles for the final position
                PlantInverseKnimatics = self.robot1.model.ikcon(requiredTranslation, current_position);
                
                %finds the required step to animate the robot
                Plant_trajectory = jtraj(current_position, PlantInverseKnimatics, steps);
                
                %loops through animation
                for j = 1:size(Plant_trajectory,1)
                    x = Plant_trajectory(j,:);
                    self.robot1.model.animate(x);
                    pause(0.1);
              
                end
        
                %deletes the brick from the start position to prepare for it to
                %move with the ar,
                
%                 delete(self.Plants(i))
                
                
           
        
            % ---------------------------------------------------------------------------------------
                %This section deliever the plant to the table position
              
                %updates the current position
                current_position = self.robot1.model.getpos();
            
                %updates the internal variable for the final position with offset
                Goal = self.Table_Position(i,:);
                Goal(3) = Goal(3) + offset;
        
                %calculate the translation, invserse kinematics, and trajectory
                requiredTranslation = transl(Goal) * troty(90,'deg')*trotx(90,'deg');
                PlantInverseKnimatics = self.robot1.model.ikcon(requiredTranslation, current_position);
                Brick_trajectory = jtraj(current_position, PlantInverseKnimatics, steps);
                
                %loops through the animation
                for k = 1:size(Brick_trajectory,1)
                    x = Brick_trajectory(k,:);
                    self.robot1.model.animate(x);
                    pause(0.25);
        
              
                    %
%                     Brick_pose = robot.model.fkine(Brick_trajectory(k,:));
%                     Brick_pose.t(3) = Brick_pose.t(3) - brickoffset;
%                     delete(Plants(i))
                    
                    drawnow();  
                
        %       % cow method -----------
        %             cow.cowModel{1}.base = robot.model.fkine(robot.model.getpos());
        %             cow.cowModel{1}.animate(0);
        %             drawnow();
        
          
                end
                
                 
                [f,v,data] = plyread('Plant.ply','tri');
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                Goal = self.Table_Position(i,:);
                trisurf(f,v(:,1)+ Goal(1),v(:,2)+ Goal(2), v(:,3) +Goal(3)...
                    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end


        end
        
        function Populate_variables(self)
            self.Table_Position = zeros(6,3);
            self.Plant_Position = zeros(6,3);
            
            self.Plant_Position(1,:) =[0.12,0.6, 0.3];
            self.Plant_Position(2,:) =[0.02, 0.6, 0.3];
            self.Plant_Position(3,:) =[-0.1,0.6,0.3];
            self.Plant_Position(4,:) =[-0.2, 0.6, 0.3];
            self.Plant_Position(5,:) =[-0.3, 0.6, 0.3];
            self.Plant_Position(6,:) =[-0.4, 0.6, 0.3];
            self.Plant_Position(7,:) =[-0.5,0.6,0.6];
            self.Plant_Position(8,:) =[-0.6,0.6,0.6];
            self.Plant_Position(9,:) =[-0.7,0.6,0.6];

            
            self.Table_Position(1,:) =[0.25, 0.15, 0.5];
            self.Table_Position(2,:) =[0.25, 0,0.5];
            self.Table_Position(3,:) =[0.25,-0.15,0.5];
            self.Table_Position(4,:) =[0.25,0.15,0.525];
            self.Table_Position(5,:) =[0.25,0,0.525];
            self.Table_Position(6,:) =[0.25,-0.15,0.525];
            self.Table_Position(7,:) =[0.25,0.15,0.55];
            self.Table_Position(8,:) =[0.25,0,0.55];
            self.Table_Position(9,:) =[0.25,-0.15,0.55]; 

            
            

        end

    end
end