classdef Lab2ClassTestBackup <handle
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
        movingCan

        %estop
        state
        estop
        Arduino

        offset_z = 0.15
        offset_y
        
    end

    methods
    %%Main !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        function self = Lab2ClassTestBackup()
            clf;
            hold on;
            SpawnGreenhouseEnvironment();
            
            %UR5 initialisation
            self.robot1 = LinearUR5;
            self.robot1.model;
            self.UR5Grip = Grippertogether;

            %A0509 initialisation
            baseTr = [1,0,0,1; 0,1,0,-0.5; 0,0,1,0.55; 0,0,0,1];
            self.robot2 = A0509(baseTr);
            self.A0509Grip = Grippertogether;
            drawnow()
            
%             self.Arduino = ArduinoClass;

            %initialise workspace variables and dynamic objects.
            self.Populate_variables(self)
            self.placePlants(self);
            self.i = 0;

            self.coopMovement(self)

        end

    end

    methods(Static)
        
        function coopMovement(self)
            q0 = [0,0,0,0,pi/2,0];
            wateringPoses = zeros(6,6);
            for i=1:size(self.watering)
                wateringPoses(i,:) = self.robot2.model.ikcon(transl(self.watering(i,1)-0.5,self.watering(i,2),self.watering(i,3)+0.6)*trotx(pi)*troty(-pi/2));
            end

          
            self.i = 1;
            self.movingCan = false;
            
            for i=1:size(self.plants)
                i
                %for the first plant zet up
                if i==1
                    self.i = i;
                    waterCanPose = self.robot2.model.ikcon(transl(self.waterCan)*trotz(-pi/2)*troty(pi/2));
                    qPath2 = jtraj(self.robot2.model.getpos,waterCanPose,50);
                    %1
                    inital_goal = self.plants(1,:);
                    final_goal = self.watering(1,:);
                    current_position = self.robot1.model.getpos();
                    current_position = (self.robot1.model.fkine(current_position).t)';
                    movement = self.CalcjtrajAttempt2(self, current_position, inital_goal, final_goal);
                    
                    qPath = self.Jtraj_inital_start;
                    self.animateBoth(self, qPath, qPath2, true, true, false);
                    self.movingCan = true;
                    pause(0.5)
        
                    qPath = self.Jtraj_start_finish;
                    qPath2 = jtraj(self.robot2.model.getpos,q0,50);
                    self.animateBoth(self, qPath, qPath2, true, true, true);            
                    pause(0.5)
        
                    qPath = self.Jtraj_finish_rest;
                    qPath2 = jtraj(self.robot2.model.getpos,wateringPoses(1,:),50);
                    self.animateBoth(self, qPath, qPath2, true, true, false); % r1 rests, r2 waters1 just placed plant
                    pause(0.5)

                    rotEndEff = self.robot2.model.getpos;
                    rotEndEff(6) = -pi/2;
                    qPath2 = jtraj(self.robot2.model.getpos,rotEndEff,25);
                    self.animateBoth(self, qPath, qPath2, false, true, false); % r1 nothing, r2 rotates endeff to waters1
                    rotEndEff = self.robot2.model.getpos;
                    rotEndEff(6) = 0;
                    qPath2 = jtraj(self.robot2.model.getpos,rotEndEff,25);
                    self.animateBoth(self, qPath, qPath2, false, true, false); % r1 nothing, r2 rotates endeff to waters1
                    % pause(0.5)
                    continue;
                end
                %plant 2-9

                self.i = self.i + 1;
                inital_goal = self.plants(i,:);
                final_goal = self.watering(i,:);

                %adding offset to the motion
                inital_goal(1) = inital_goal(1) - 0.1;
                final_goal(1) = final_goal(1) + 0.1;
                inital_goal(3) = inital_goal(3) + self.offset_z;
                final_goal(3) = final_goal(3) + self.offset_z;

                
                % current_position = self.robot1.model.getpos();
                % current_position = (self.robot1.model.fkine(current_position).t)';
                % movement = self.CalcjtrajAttempt2(self, current_position, inital_goal, final_goal, 1);
                
                % qPath = self.Jtraj_inital_start;
                qinitial = self.robot1.model.ikcon(transl(inital_goal)*trotx(pi/2)*troty(pi/2));
                qPath = jtraj(self.robot1.model.getpos,qinitial,50);
                qPath2 = jtraj(self.robot2.model.getpos,q0,50);
                self.animateBoth(self, qPath, qPath2, true, true, false); % r1 fetches new plant2, r2 rests
                pause(0.5)
    
                %close gripper              
                self.UR5Grip.closeGripper(self.robot1.model.fkine(self.robot1.model.getpos()));
                %%%%%%%%%%%add interminde point here
                midpoint = [(final_goal(1)- 0.5),final_goal(2), inital_goal(3)];
%                 M = [1,1,0,0,0,0]; --- masking doesn't work with a 6 or 7dof robot
%                 qstart = self.robot1.model.ikine((transl(midpoint)*trotx(pi/2)*troty(pi/2)),'mask',M, 'forceSoln');
                qstart = self.robot1.model.ikcon((transl(midpoint)*trotx(pi/2)*troty(pi/2)));
                qfinsh = self.robot1.model.ikcon(transl(final_goal)*trotx(pi/2)*troty(pi/2),qstart);
                
                qPathstart = jtraj(self.robot1.model.getpos,qstart,25);
                qPathfinish = jtraj(qstart,qfinsh,25);

                qPath = [qPathstart; qPathfinish];



%                 qstart = self.robot1.model.ikcon(transl(final_goal)*trotx(pi/2)*troty(pi/2));
%                 qPath = jtraj(self.robot1.model.getpos,qstart,50);
                % qPath2 = jtraj(self.robot2.model.getpos,wateringPoses(2,:),50);
                self.animateBoth(self, qPath, qPath2, true, false, true); % r1 places plant on table, r2 nothing
                pause(0.5)
                self.UR5Grip.OpenGripper(self.robot1.model.fkine(self.robot1.model.getpos()));
    
                qrest = self.robot1.model.ikcon(transl(final_goal(1),final_goal(2),final_goal(3)+0.5)*trotx(pi/2)*troty(pi/2));
                qPath = jtraj(self.robot1.model.getpos,qrest,50);
                % qPath2 = jtraj(self.robot2.model.getpos,q0,50);
                self.animateBoth(self, qPath, qPath2, true, false, false); % r1 rest, r2 nothing
                % pause(0.5)
                return_to_table = self.watering(i-1,:);
                
                %ADD OFFSETS
                %adding offset to the motion
                return_to_table(1) = return_to_table(1) + 0.2;      
                return_to_table(3) = return_to_table(3) + self.offset_z+0.05;


                qinitial = self.robot1.model.ikcon(transl(return_to_table)*trotx(pi/2)*troty(pi/2));
                qPath = jtraj(self.robot1.model.getpos,qinitial,50);
                % qPath2 = jtraj(self.robot2.model.getpos,wateringPoses(1,:),50);
                self.animateBoth(self, qPath, qPath2, true, false, false); % r1 goes back to plant1 on table, r2 nothing
                pause(0.5)
                self.UR5Grip.closeGripper(self.robot1.model.fkine(self.robot1.model.getpos()));
                
                self.i = self.i - 1; %take the previous plantObject back to shelf
                return_to_shelf = self.plants(i-1,:);
                
                %ADD OFFSETS
                 %adding offset to the motion
                return_to_shelf(1) = return_to_shelf(1) - 0.1;
                return_to_shelf(3) = return_to_shelf(3) + self.offset_z+0.35;

                qstart = self.robot1.model.ikcon(transl(return_to_shelf)*trotx(pi/2)*troty(pi/2));
                qPath = jtraj(self.robot1.model.getpos,qstart,50);
                % qPath2 = jtraj(self.robot2.model.getpos,wateringPoses(2,:),50);
                self.animateBoth(self, qPath, qPath2, true, false, true); % r1 goes back to shelf1 with plant, r2 nothing
                pause(0.5)
                self.i = self.i + 1; %increment for next loop
                self.UR5Grip.OpenGripper(self.robot1.model.fkine(self.robot1.model.getpos));
                qrest = self.robot1.model.ikcon(transl([0,0.25,0.6])*trotx(pi/2)*troty(pi/2));
                qPath = jtraj(self.robot1.model.getpos,qrest,50);
                qPath2 = jtraj(self.robot2.model.getpos,wateringPoses(i,:),50);
                self.animateBoth(self, qPath, qPath2, true, true, false); % r1 rests, r2 water2
                pause(0.5)

                qPath = jtraj(self.robot1.model.getpos,qrest,25); % gener
                rotEndEff = self.robot2.model.getpos;
                rotEndEff(6) = -pi/2;
                qPath2 = jtraj(self.robot2.model.getpos,rotEndEff,25);
                self.animateBoth(self, qPath, qPath2, false, true, false); % r1 nothing, r2 rotates endeff to waters1
                rotEndEff = self.robot2.model.getpos;
                rotEndEff(6) = 0;
                qPath2 = jtraj(self.robot2.model.getpos,rotEndEff,25);
                self.animateBoth(self, qPath, qPath2, false, true, false); % r1 nothing, r2 rotates endeff to waters1
                % pause(0.15)
            end

            end_goal = self.plants(6,:);
            % end_goal(3) = end_goal(3) + 0.4;
            end_goal(1) = end_goal(3) - 0.1;
            end_goal(3) = end_goal(3) + self.offset_z+0.35;
            table_goal = self.watering(6,:);

            qPath2 = jtraj(self.robot2.model.getpos,q0,50);
            self.animateBoth(self, qPath, qPath2, false, true, false); % r1 nothing, r2 rests
            % pause(0.5)

            qinitial = self.robot1.model.ikcon(transl(table_goal)*trotx(pi/2)*troty(pi/2));
            qPath = jtraj(self.robot1.model.getpos,qinitial,50);
            self.animateBoth(self, qPath, qPath2, true, false, false); % r1 picks up final plant, r2 nothing
            pause(0.5)
            
            qstart = self.robot1.model.ikcon(transl(end_goal)*trotx(pi/2)*troty(pi/2));
            qPath = jtraj(self.robot1.model.getpos,qstart,50);
            self.animateBoth(self, qPath, qPath2, true, false, true); % r1 returns final plant, r2 nothing
            pause(0.5)

            qPath = jtraj(self.robot1.model.getpos,[-0.5,0,0,0,0,0,0],50);
            self.animateBoth(self, qPath, qPath2, true, false, false); % return to q0

            
        end



        function animateBoth(self, qPath, qPath2, flag, flag2, carryingPlant)
            loop=[]; %handling loop size depending on flags
            if flag == false
                loop = qPath2;
            elseif flag
                loop = qPath;
            end
            if flag2 == false
                loop = qPath;
            elseif flag2
                loop = qPath2;
            end

            for i=1:size(loop) % qPath of both needs to be of same size
                % check estop!!!!!!!
                % self.Arduino.ReadArduino(self.state)
                self.estop = getEstopStatus();

                while self.estop == 1
                    self.estop = getEstopStatus();
                    self.Arduino.ReadArduino(self.state)
                end

                if flag
                    self.robot1.model.animate(qPath(i,:));
                end
                if flag2
                    self.robot2.model.animate(qPath2(i,:));
                    if self.movingCan
                        tr = self.robot2.model.fkine(self.robot2.model.getpos).T;
                        tr = tr * troty(-pi/2) * trotz(pi/2);
                        self.updatedWateringCanVerts = [self.wateringCanVertices,ones(size(self.wateringCanVertices,1),1)] * tr';
                        set(self.wateringCanObjects{1},'Vertices',self.updatedWateringCanVerts(:,1:3)); % Updates can position to end effector transform
                        drawnow();
                    end
                end

                if carryingPlant
                    tr = self.robot1.model.fkine(self.robot1.model.getpos).T;
                    tr = tr * trotx(-pi/2);
                    tr(3,4) = tr(3,4) - 0.1; % Z offset
                    tr(1,4) = tr(1,4) + 0.1; % x offset axis of endereffector
%                     tr(2,4) = tr(2,4) - 0.2; % x offset left/right of endereffector
                    
                    self.updatedPlantVerts = [self.plantVertices,ones(size(self.plantVertices,1),1)] * tr';
                    set(self.plantObjects{self.i},'Vertices',self.updatedPlantVerts(:,1:3)); % Updates plant position to end effector transform
                end
                self.A0509Grip.MoveGripper(self.robot2.model.fkine(self.robot2.model.getpos()));
                self.UR5Grip.MoveGripper(self.robot1.model.fkine(self.robot1.model.getpos()));   
                drawnow();
            end
            if carryingPlant
                self.updatedPlantVerts = [self.plantVertices,ones(size(self.plantVertices,1),1)] * tr';
                set(self.plantObjects{self.i},'Vertices',self.updatedPlantVerts(:,1:3)); % Updates plant position to end effector transform
                drawnow();
                % selfI = self.i
            end            
        end

        function placePlants(self)
            % hold on; 
            % for i= 1:size(self.Plant_Position, 1)
            %     self.Plants(i) = PlaceObject(['Plant.ply'], [self.Plant_Position(i,:)]);
            % end
            % self.Watering_Can = PlaceObject(['HalfSizedRedGreenBrick.ply'], [0.25, 0.15, 0.5])
            % 
            % camlight;
            % hold on;

            self.plantObjects = cell(6,1);
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
        
        
 
        
        
        function Populate_variables(self)
            % self.Table_Position = zeros(6,3);
            % self.Plant_Position = zeros(6,3);
            % 
            % self.Plant_Position(1,:) =[0.12,0.6, 0.31];
            % self.Plant_Position(2,:) =[0.02, 0.6, 0.31];
            % self.Plant_Position(3,:) =[-0.1,0.6,0.31];
            % self.Plant_Position(4,:) =[-0.2, 0.6, 0.31];
            % self.Plant_Position(5,:) =[-0.3, 0.6, 0.31];
            % self.Plant_Position(6,:) =[-0.4, 0.6, 0.31];
            % self.Plant_Position(7,:) =[-0.5,0.6,0.6];
            % self.Plant_Position(8,:) =[-0.6,0.6,0.6];
            % self.Plant_Position(9,:) =[-0.7,0.6,0.6];
            
            % self.plant1 = [0.15,0.6, 0.31];
            % self.plant2 = [-0.15, 0.6, 0.31];
            % self.plant3 = [-0.45,0.6,0.31];
            % self.plant4 = [0.15, 0.6, 0.71];
            % self.plant5 = [-0.15, 0.6, 0.71];
            % self.plant6 = [-0.45, 0.6, 0.71];
            self.plant1 = [0.1, 0.5, 0.34];
            self.plant2 = [-0.05, 0.5, 0.34];
            self.plant3 = [-0.2, 0.5,0.34];
            self.plant4 = [-0.35, 0.5, 0.34];
            self.plant5 = [-0.5, 0.5, 0.34];
            self.plant6 = [-0.65, 0.5, 0.34];
            % self.plant7 = [-0.5,0.6,0.6];
            % self.plant8 = [-0.6,0.6,0.6];
            % self.plant9 = [-0.7,0.6,0.6];
            self.plants = [self.plant1;self.plant2;self.plant3;self.plant4;self.plant5;self.plant6];%;self.plant7;self.plant8;self.plant9];
   
            
            % self.Table_Position(1,:) =[0.25, 0.15, 0.5];
            % self.Table_Position(2,:) =[0.25, 0.15,0.5];
            % self.Table_Position(3,:) =[0.25,0.15,0.5];
            % self.Table_Position(4,:) =[0.25,0.15,0.525];
            % self.Table_Position(5,:) =[0.25,0.15,0.525];
            % self.Table_Position(6,:) =[0.25,0.15,0.525];
            % self.Table_Position(7,:) =[0.25,0.15,0.55];
            % self.Table_Position(8,:) =[0.25,0.15,0.55];
            % self.Table_Position(9,:) =[0.25,0.15,0.55];

            self.wateringPos1 = [0.5, -0.4, 0.5+0.05];
            % self.wateringPos2 = [0.5,  -0.4,   0.5+0.05];
            self.wateringPos2 = [0.5,  0.1,   0.5+0.05];
            self.watering = [self.wateringPos1; self.wateringPos2; self.wateringPos1; self.wateringPos2; self.wateringPos1; self.wateringPos2];
            

            self.waterCan = [0.7, -1, 0.5+0.05];%[1, -1.2, 0.5+0.05];
            self.waterCans = [self.waterCan];
            self.state = 0;
            self.estop = 0;
        end



        function [trajectory] = CalcjtrajAttempt2(self, CurrentPosition, start, finish)
            rest = finish;
            rest(3) = rest(3)+0.5;
            steps = 50;
            
            start(1) = start(1) - 0.1;
            finish(1) = finish(1) - 0.1;
            start(3) = start(3) + self.offset_z;
            finish(3) = finish(3) + self.offset_z+0.1;

            qinital = self.robot1.model.ikcon(transl(CurrentPosition)*trotx(pi/2)*troty(pi/2)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
            qstart = self.robot1.model.ikcon(transl(start)*trotx(pi/2)*troty(pi/2)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
            qfinish = self.robot1.model.ikcon(transl(finish)*trotx(pi/2)*troty(pi/2));
            qrest = self.robot1.model.ikcon(transl(rest)*trotx(pi/2)*troty(pi/2));
   
            self.Jtraj_inital_start = jtraj(qinital,qstart,steps);
            self.Jtraj_start_finish = jtraj(qstart,qfinish,steps);
            self.Jtraj_finish_rest = jtraj(qfinish,qrest,steps);
            trajectory = [self.Jtraj_inital_start; self.Jtraj_start_finish; self.Jtraj_finish_rest]; % adding all joint angles to a single qMatrix            
        end

    end

end