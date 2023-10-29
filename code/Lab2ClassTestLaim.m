classdef Lab2ClassTestLaim <handle
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
        Watering_Can
        Jtraj_inital_start
        Jtraj_start_finish
        Jtraj_finish_rest
        plant1
        plant2
        plant3
        plant4
        plant5
        plant6
        plant7
        plant8
        plant9
        plants

        plantObjects
        plantVertices
        updatedPlantVerts
        endPlants


        watering
        wateringPos1
        wateringPos2
        wateringCanObjects
        wateringCanVertices
        updatedWateringCanVerts
        endwateringCans
        waterCan
        waterCans
        
    end

    methods
    %%Main !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        function self = Lab2ClassTestLaim()
            clf;
            hold on;
            SpawnGreenhouseEnvironment();
            
            %UR5 initialisation
            self.robot1 = LinearUR5;
            self.robot1.model;
            %self.UR5Grip = UR5_Gripper;
            
            
            
            %A0509 initialisation
            baseTr = [1,0,0,1.25; 0,1,0,-0.5; 0,0,1,0.55; 0,0,0,1];
            self.robot2 = A0509(baseTr);
            % self.robot2.model.base = self.robot2.model.base.T * trotz(pi);
            % self.robot2.PlotAndColourRobot();
            %self.A0509Grip = A0509_Gripper;
            drawnow()
            
                
            self.Populate_variables(self)
            self.placePlants(self);


            self.i = 0;
            for i=1:size(self.plants)

                inital_goal = self.plants(i,:);
                final_goal = self.watering(i,:);
        
                self.PlaceOnePlantOnTable(self, inital_goal,final_goal, i)
                self.waterPlant(self, i)
                self.PlaceOnePlantOnTable(self, final_goal,inital_goal, i)
               
            end

           

        end

    end

    methods(Static)
        
        function placePlants(self)
            % hold on; 
            % for i= 1:size(self.Plant_Position, 1)
            %     self.Plants(i) = PlaceObject(['Plant.ply'], [self.Plant_Position(i,:)]);
            % end
            % self.Watering_Can = PlaceObject(['HalfSizedRedGreenBrick.ply'], [0.25, 0.15, 0.5])
            % 
            % camlight;
            % hold on;

            self.plantObjects = cell(9,1);
            for i=1:size(self.plants)
                self.plantObjects{i} = PlaceObject('Plant.ply'); % Placing plants into world
                self.plantVertices = get(self.plantObjects{i},'Vertices');
                transform = [self.plantVertices,ones(size(self.plantVertices,1),1)] * transl(self.plants(i,:))';
                set(self.plantObjects{i},'Vertices',transform(:,1:3));
            end
            
            self.wateringCanObjects = cell(1,1);
            %self.wateringCanObjects = PlaceObject('Watering_can.ply',[1, -1.2, 0.5+0.05]); % Placing plants into world
            self.wateringCanObjects{1} = PlaceObject('Watering_can.ply');
            self.wateringCanVertices = get(self.wateringCanObjects{1},'Vertices');
            %self.wateringCanVertices = [get(self.wateringCanObjects,'Vertices'), ones(size(get(self.wateringCanObjects,'Vertices'),1),1)] * trotz(pi/2);
            transform = [self.wateringCanVertices,ones(size(self.wateringCanVertices,1),1)] * transl(self.waterCan)';
            %set(self.wateringCanObjects,'Vertices',self.wateringCanVertices(:,1:3));
            set(self.wateringCanObjects{1},'Vertices',transform(:,1:3));
        end

   


        

        function PlaceOnePlantOnTable(self, inital_goal, Final_goal, count)
            %% placing plants on table
            offset = 0;
            
            inital_goal(3) = inital_goal(3) + offset;             
            Final_goal(3) = Final_goal(3) + offset;
            

            current_position = (self.robot1.model.getpos());
            current_position = (self.robot1.model.fkine(current_position).t)';

            [Jtraj_inital_start, Jtraj_start_finish, Jtraj_finish_rest] = self.CalcjtrajAttempt2(self, current_position, inital_goal, Final_goal, 1);
            endPlantObjects = [];

            %for i=1:size(brickPoses)
                %count = i;                    
                %qPath = jtraj(r.model.getpos,brickPoses(i,:),100); % Creates path of robot current pos to brick at index i
                animateRobot(Jtraj_inital_start,self.robot1); % Steps over the qPath and animates the robot, takes a flag to indicate whether the brick is being picked up
                posR1shelf = transl(self.robot1.model.fkine(self.robot1.model.getpos))
                % fkine = self.robot1.model.fkine(self.robot1.model.getpos);
                % translateR1 = transl(fkine);
                pause(0.5);
            
                %qPath = jtraj(r.model.getpos,wallPoses(i,:),100); % Creates path of robot current pos (previous plant start) to dropoff point at index i
                % animateRobot(qPath,r);
                for j=1:size(Jtraj_start_finish)
                    self.robot1.model.animate(Jtraj_start_finish(j,:));
                    tr = self.robot1.model.fkine(self.robot1.model.getpos).T;
                    tr = tr * trotx(-pi/2);
                    self.updatedPlantVerts = [self.plantVertices,ones(size(self.plantVertices,1),1)] * tr';
                    set(self.plantObjects{count},'Vertices',self.updatedPlantVerts(:,1:3)); % Updates plant position to end effector transform
                    drawnow();
                end
                disp(' ');
                disp(['Current joint values: [', num2str(Jtraj_start_finish(count,:)), ']']); % Prints joint values after finishing motion
                
                self.updatedPlantVerts = [self.plantVertices,ones(size(self.plantVertices,1),1)] * tr';
                set(self.plantObjects{count},'Vertices',self.updatedPlantVerts(:,1:3)); % Updates plant position to end effector transform
                %endPlantObjects(:,end+1) = [PlaceObject('Plant.ply',self.watering(i,:))];
                drawnow();
                posR1dropoff = transl(self.robot1.model.fkine(self.robot1.model.getpos))
                pause(0.5);
                %disp(['Place plant down at: [', num2str(pos2), '] Moving to: [', num2str(pos3), ']']);
                animateRobot(Jtraj_finish_rest,self.robot1);
                pause(0.5);
            %end
            % qPath = jtraj(r.model.getpos,q0,50); % Resets position for final movement
            % animateRobot(qPath,
            %loops through animation
            % for j = 1:size(movement,1)
            %     Plant_trajectory = movement(j,:)
            %     for k = 1:size(Plant_trajectory,1)
            %         x = Plant_trajectory(k,:);
            %         self.robot1.model.animate(x);
            %         pause(0.25);
            %         if j > 50 && j < 100
            %             delete(self.Plants(i))
            %             RobotEndeffector = (self.robot1.model.fkine(x).t)';
            %             self.Plants(i) = PlaceObject('Plant.ply', RobotEndeffector);                           
            %             drawnow()
            % 
            %         end
            %         j                                                                       
            %     end                                                   
            % end
            % delete(self.Plants(i))
            % self.Plants(i) = PlaceObject('Plant.ply', Final_goal);
            % delete(self.Plants(i))
            % Final_goal(3) = Final_goal(3) - offset;
            % self.Plants(i) = PlaceObject('Plant.ply', Final_goal);
        end
        



        function waterPlant(self, count)
            % wateringPos1 = [0.75, -0.4, 0.5+0.05];
            % wateringPos2 = [0.75,  0,   0.5+0.05];
            % watering = [wateringPos1; wateringPos2; wateringPos1; wateringPos2; wateringPos1; wateringPos2];
            q0 = [0,0,0,0,0,0];
            wateringPoses = zeros(6,6);
            %shelfPoses = zeros(9,6);
            for i=1:size(self.watering)
                wateringPoses(i,:) = self.robot2.model.ikcon(transl(self.watering(i,:))*trotx(pi));%*troty(-pi/2)); %*trotx(pi));
                %shelfPoses(i,:) = r.model.ikcon(transl(e.endBricks(i,:))*trotx(pi));
            end
            
            axis([-2,2,-2,2,0,2])

            if count == 1
                waterCanPose = self.robot2.model.ikcon(transl(self.waterCan)*trotz(-pi/2)*troty(pi/2));
                qPath = jtraj(self.robot2.model.getpos,waterCanPose,100);
                animateRobot(qPath,self.robot2);
                pause(0.5)
                
                qPath = jtraj(self.robot2.model.getpos,q0,50);
                for j=1:size(qPath)
                    self.robot2.model.animate(qPath(j,:))
                    tr = self.robot2.model.fkine(self.robot2.model.getpos).T;
                    tr = tr * troty(-pi/2) * trotz(pi/2); %trotx(-pi/2);
                    self.updatedWateringCanVerts = [self.wateringCanVertices,ones(size(self.wateringCanVertices,1),1)] * tr';
                    set(self.wateringCanObjects{1},'Vertices',self.updatedWateringCanVerts(:,1:3)); % Updates can position to end effector transform
                    drawnow();
                end
                pause(0.5);
            end
            M = [1,1,1,1,1,1];
            wateringPoseTemp = self.watering(1,:);
            wateringPoseTemp(3)= wateringPoseTemp(3)+ 0.25;
            plot3(wateringPoseTemp(1),wateringPoseTemp(2),wateringPoseTemp(3),'o')
%             wateringPoseTemp = self.robot2.model.ikine(transl(wateringPoseTemp)*trotx(pi), 'mask',M, 'forceSoln');
            wateringPoseTemp = self.robot2.model.ikcon(transl(wateringPoseTemp)*trotx(pi));
       

            qPath = jtraj(self.robot2.model.getpos,wateringPoseTemp,100); % Creates path of robot current pos to brick at index i
            for j=1:size(qPath)
                self.robot2.model.animate(qPath(j,:)) 
                tr = self.robot2.model.fkine(self.robot2.model.getpos).T;
                % tr = tr * trotx(-pi/2);
                tr = tr * troty(-pi/2) * trotz(pi/2); % undo the pose translation in reverse. ie watercanpose was z,y, so undo with -y,-z
                self.updatedWateringCanVerts = [self.wateringCanVertices,ones(size(self.wateringCanVertices,1),1)] * tr';
                set(self.wateringCanObjects{1},'Vertices',self.updatedWateringCanVerts(:,1:3)); % Updates can position to end effector transform
                drawnow();
            end
            pause(0.5);

            wateringPoseTemp = self.watering(1,:);
            wateringPoseTemp(3)= wateringPoseTemp(3)+ 0.6;
            wateringPoseTemp(1)= wateringPoseTemp(1)+ 0.4;
            plot3(wateringPoseTemp(1),wateringPoseTemp(2),wateringPoseTemp(3),'o')
%           wateringPoseTemp = self.robot2.model.ikine(transl(wateringPoseTemp)*trotx(pi), 'mask',M, 'forceSoln');
            q0 = self.robot2.model.ikcon(transl(wateringPoseTemp)*trotx(pi));
            qPath = jtraj(self.robot2.model.getpos,q0,50);
            for j=1:size(qPath)
                self.robot2.model.animate(qPath(j,:)) 
                tr = self.robot2.model.fkine(self.robot2.model.getpos).T;
                % tr = tr * trotx(-pi/2);
                tr = tr * troty(-pi/2) * trotz(pi/2);
                self.updatedWateringCanVerts = [self.wateringCanVertices,ones(size(self.wateringCanVertices,1),1)] * tr';
                set(self.wateringCanObjects{1},'Vertices',self.updatedWateringCanVerts(:,1:3)); % Updates can position to end effector transform
                drawnow();
            end
            pause(0.5);

            % %for i=1:size(wateringPoses)
            %     qPath = jtraj(self.robot2.model.getpos,wateringPoses(count,:),100); % Creates path of robot current pos to brick at index i
            %     % for j=1:size(qPath)
            %     %     self.robot2.model.animate(qPath(j,:));
            %     % 
            %     %     drawnow();
            %     % end
            %     animateRobot(qPath,self.robot2);
            %     pause(0.5);
            % 
            %     qPath = jtraj(self.robot2.model.getpos,q0,50);
            %     animateRobot(qPath,self.robot2);
            % 
            %     pos = transl(self.robot2.model.fkine(self.robot2.model.getpos));
            %     %fkine = aR.model.fkine(aR.model.getpos);
            %     %translate = transl(fkine)
            %     pause(0.5)
            % %end
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
                
                self.plant1 = [0.12,0.6, 0.3];
                self.plant2 = [0.02, 0.6, 0.3];
                self.plant3 = [-0.1,0.6,0.3];
                self.plant4 = [-0.2, 0.6, 0.3];
                self.plant5 = [-0.3, 0.6, 0.3];
                self.plant6 = [-0.4, 0.6, 0.3];
                self.plant7 = [-0.5,0.6,0.6];
                self.plant8 = [-0.6,0.6,0.6];
                self.plant9 = [-0.7,0.6,0.6];
                self.plants = [self.plant1;self.plant2;self.plant3;self.plant4;self.plant5;self.plant6];%;self.plant7;self.plant8;self.plant9];
    
                
                self.Table_Position(1,:) =[0.25, 0.15, 0.5];
                self.Table_Position(2,:) =[0.25, 0,0.5];
                self.Table_Position(3,:) =[0.25,-0.15,0.5];
                self.Table_Position(4,:) =[0.25,0.15,0.525];
                self.Table_Position(5,:) =[0.25,0,0.525];
                self.Table_Position(6,:) =[0.25,-0.15,0.525];
                self.Table_Position(7,:) =[0.25,0.15,0.55];
                self.Table_Position(8,:) =[0.25,0,0.55];
                self.Table_Position(9,:) =[0.25,-0.15,0.55];

                self.wateringPos1 = [0.5, -0.4, 0.5+0.05];
                self.wateringPos2 = [0.5,  0.1,   0.5+0.05];
                self.watering = [self.wateringPos1; self.wateringPos2; self.wateringPos1; self.wateringPos2; self.wateringPos1; self.wateringPos2];
                
                self.waterCan = [0.7, -1, 0.5+0.05];%[1, -1.2, 0.5+0.05];
                self.waterCans = [self.waterCan];
            end



            function [Jtraj_inital_start, Jtraj_start_finish, Jtraj_finish_rest] = CalcjtrajAttempt2(self, CurrentPosition, start, finish, robot)
                rest = finish;
                rest(3) = rest(3)+0.5;
                start_off_shelf = start;
                start_off_shelf(2) = start_off_shelf(2)-0.2;
                steps = 50;
                M = [1,1,1,1,1,1];
                if robot == 2
                    
                    qinital = self.robot2.model.ikcon(transl(CurrentPosition)*trotx(pi)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
                    qstart = self.robot2.model.ikcon(transl(start)*trotx(pi/2)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
                    qfinish = self.robot2.model.ikcon(transl(finish)*trotx(pi/2));
                    qrest = self.robot2.model.ikcon(transl(rest)*trotx(pi/2));                   
                    
                else
                    qinital = self.robot1.model.ikcon(transl(CurrentPosition)*trotx(pi/2)*troty(pi/2)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
                    qstart_shelf = self.robot1.model.ikcon(transl(start_off_shelf)*trotx(pi/2)*troty(pi/2)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
                    qstart = self.robot1.model.ikcon(transl(start)*trotx(pi/2)*troty(pi/2)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
                    qfinish = self.robot1.model.ikcon(transl(finish)*trotx(pi/2)*troty(pi/2));
                    qrest = self.robot1.model.ikcon(transl(rest)*trotx(pi/2)*troty(pi/2));

%                          qinital = self.robot1.model.ikine(transl(CurrentPosition)*trotx(pi/2)*troty(pi/2),'mask',M, 'forceSoln'); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
%                     qstart = self.robot1.model.ikine(transl(start)*trotx(pi/2)*troty(pi/2),'mask',M, 'forceSoln'); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
%                     qfinish = self.robot1.model.ikine(transl(finish)*trotx(pi/2)*troty(pi/2),'mask',M, 'forceSoln');
%                     qrest = self.robot1.model.ikine(transl(rest)*trotx(pi/2)*troty(pi/2),'mask',M, 'forceSoln');
                    
                    

                end

                Jtraj_inital_start = jtraj(qinital,qstart,steps);

                
                
                temp = jtraj(qstart, qstart_shelf, 20);
                Jtraj_start_finish = jtraj(qstart_shelf,qfinish,steps);
                Jtraj_start_finish = [temp; Jtraj_start_finish];
                Jtraj_finish_rest = jtraj(qfinish,qrest,steps);
%                 trajectory = [self.Jtraj_inital_start; self.Jtraj_start_finish; self.Jtraj_finish_rest]; % adding all joint angles to a single qMatrix
                
            end

        end

    end