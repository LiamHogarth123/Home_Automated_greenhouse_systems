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
        Watering_Can
    end

    methods
    
        function self = Lab2ClassTest()
            clf;
            hold on;
            SpawnGreenhouseEnvironment();
            self.robot1 = LinearUR5;
            self.robot1.model;
            %self.UR5Grip = UR5_Gripper;
            self.robot2 = A0509;
            
            drawnow()
%             self.A0509Grip = A0509_Gripper;
            self.Populate_variables(self)
            
            self.placePlants(self);
%             self.PlacePlantOnTable(self);
            self.i = 0;
            for i=1:size(self.Plant_Position, 1)
                inital_goal = self.Plant_Position(i,:);
                final_goal = self.Table_Position(i,:);
                self.PlaceOnePlantOnTable(self, inital_goal,final_goal, i)
                watering_can_position = [0.25, 0.15, 0.5];
                self.waterPlant(self, watering_can_position, final_goal, i)
%                 self.PlaceOnePlantOnTable(self, final_goal,inital_goal, i)
%                
            end

           

        end

    end

    methods(Static)
        
        function placePlants(self)
            hold on; 
            for i= 1:size(self.Plant_Position, 1)
                self.Plants(i) = PlaceObject(['Plant.ply'], [self.Plant_Position(i,:)]);
            end
            self.Watering_Can = PlaceObject(['HalfSizedRedGreenBrick.ply'], [0.25, 0.15, 0.5])

            camlight;
            hold on;
        end

   


        

        function PlaceOnePlantOnTable(self, inital_goal, Final_goal, i)
            %% placing plants on table
                offset = 0;
                
         
                inital_goal(3) = inital_goal(3) + offset;             
                Final_goal(3) = Final_goal(3) + offset;
                current_position = self.robot1.model.getpos();
                rest_goal = Final_goal(1) -0.4;
                
                
              
                
%                 %finds the required step to animate the robot
%                 GetplantTraj = self.Calcjtraj(self, current_position, inital_goal, 1);
%                 inital_goal = self.robot1.model.ikcon(transl(inital_goal));
%                 DelivPlant = self.Calcjtraj(self, inital_goal, Final_goal, 1);
%                 Final_goal = self.robot1.model.ikcon(transl(Final_goal));
%                 rest = self.Calcjtraj(self, Final_goal, rest_goal, 1);
%                 Movenment = [GetplantTraj;DelivPlant;rest];
                current_position = (self.robot1.model.getpos());
                current_position = (self.robot1.model.fkine(current_position).t)';

                

                Movenment = self.CalcjtrajAttempt2(self, current_position, inital_goal, Final_goal, 1);

                %loops through animation
                for j = 1:size(Movenment,1)
                    Plant_trajectory = Movenment(j,:)
                    for k = 1:size(Plant_trajectory,1)
                        x = Plant_trajectory(k,:);
                        self.robot1.model.animate(x);
                        pause(0.25);
                        if j > 50 && j < 100
                            delete(self.Plants(i))
                            RobotEndeffector = (self.robot1.model.fkine(x).t)';
                            self.Plants(i) = PlaceObject('Plant.ply', RobotEndeffector);                           
                            drawnow()
                           
                        end
                        j                                                                       
                    end                                                   
                end
                delete(self.Plants(i))
                self.Plants(i) = PlaceObject('Plant.ply', Final_goal);
                delete(self.Plants(i))
                Final_goal(3) = Final_goal(3) - offset;
                self.Plants(i) = PlaceObject('Plant.ply', Final_goal);

        end
        



        function waterPlant(self, inital_goal, Final_goal,i)
           offset = 0;
                
         
                inital_goal(3) = inital_goal(3) + offset;             
                Final_goal(3) = Final_goal(3) + offset;
                current_position = self.robot1.model.getpos();
                rest_goal = Final_goal(1) -0.4;
                
                
              
                
                %finds the required step to animate the robot
                GetplantTraj = self.Calcjtraj(self, current_position, inital_goal, 2); %error here
                inital_goal = self.robot1.model.ikcon(transl(inital_goal));
                DelivPlant = self.Calcjtraj(self, inital_goal, Final_goal, 2);
                Final_goal = self.robot1.model.ikcon(transl(Final_goal));
                rest = self.Calcjtraj(self, Final_goal, rest_goal, 2);
                Movenment = [GetplantTraj;DelivPlant;rest];
         

                %loops through animation
                for j = 1:size(Movenment,1)
                    Plant_trajectory = Movenment(j,:);
                    for k = 1:size(Plant_trajectory,1)
                        x = Plant_trajectory(k,:);
                        self.robot1.model.animate(x);
                        pause(0.25);
                        if j > 50 && j < 100
                            delete(self.Plants(i))
                            RobotEndeffector = (self.robot1.model.fkine(x).t)';
                            self.Plants(i) = PlaceObject('Plant.ply', RobotEndeffector);                           
                            drawnow()
                           
                        end
                        j                                                                       
                    end                                                   
                end
                delete(self.Plants(i))
                self.Plants(i) = PlaceObject('Plant.ply', Final_goal);
                delete(self.Plants(i))
                Final_goal(3) = Final_goal(3) - offset;
                self.Plants(i) = PlaceObject('Plant.ply', Final_goal);

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

            function [trajectory] = Calcjtraj(self, start, finish, robot)
    
                steps = 50;

                requiredTranslation = transl(finish) * troty(90,'deg')*trotx(90,'deg');
                if robot == 2
                    PlantInverseKnimatics = self.robot2.model.ikcon(requiredTranslation, start);
                else
                    PlantInverseKnimatics = self.robot1.model.ikcon(requiredTranslation, start);
                end
                trajectory = jtraj(start, PlantInverseKnimatics, steps);
                
            end

            function [trajectory] = CalcjtrajAttempt2(self, CurrentPosition, start, finish, robot)
                rest = finish;
                rest(3) = rest(3)+0.5;
                steps = 50;
                if robot == 2
                    
                    qinital = self.robot2.model.ikcon(transl(CurrentPosition)*trotx(pi)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
                    qstart = self.robot2.model.ikcon(transl(start)*trotx(pi)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
                    qfinish = self.robot2.model.ikcon(transl(finish)*trotx(pi));
                    qrest = self.robot2.model.ikcon(transl(rest)*trotx(pi));                   
                    
                else
                    qinital = self.robot1.model.ikcon(transl(CurrentPosition)*trotx(pi)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
                    qstart = self.robot1.model.ikcon(transl(start)*trotx(pi)); % uses ikcon to get joint angles and prevent collision. multiplied by trotx(pi) to rotate end-effector
                    qfinish = self.robot1.model.ikcon(transl(finish)*trotx(pi));
                    qrest = self.robot1.model.ikcon(transl(rest)*trotx(pi));

                end
                Jtraj_inital_start = jtraj(qinital,qstart,steps);
                Jtraj_start_finish = jtraj(qstart,qfinish,steps);
                Jtraj_finish_rest = jtraj(qfinish,qrest,steps);
                trajectory = [Jtraj_inital_start; Jtraj_start_finish; Jtraj_finish_rest]; % adding all joint angles to a single qMatrix
                
            end

        end

    end
