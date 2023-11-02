classdef Grippertogether
    properties(Access = public)   
        Link_One
        Link_two
        offset
    end
    
    methods
%% Constructor
        function self = Grippertogether()
            hold on
            self.Link_One = GripperLeft();
            hold on
            self.Link_two = GripperRight();
            drawnow()
            q1 = [deg2rad(25), deg2rad(25), deg2rad(29)];
            q2 = [deg2rad(-25), deg2rad(-25), deg2rad(-29)];
            self.Link_One.model.animate(q1);
            self.Link_two.model.animate(q2);
            self.offset = zeros(1,3);

            

        end

%% CreateModel
        function closeGripper(self, pos)
             for l = deg2rad(40):0.05:deg2rad(50)                          %%this for loop runs between the values of the open and close position of the first joint angle which i use to open/close the gripper
                 self.Link_two.model.base = pos.T*trotx(-pi/2); 
                 self.Link_One.model.base = pos.T*trotx(pi/2);
                 self.Link_One.model.animate([l,deg2rad(25),deg2rad(29)]);   %this animates the grippers joint angles with the l variable to open/close the gripper
                 self.Link_two.model.animate([3*pi/2-l,deg2rad(25),deg2rad(29)]);
             end
        
        end 
        
        
        function OpenGripper(self, pos)
             for l = deg2rad(50):-0.05:deg2rad(40) %%this for loop runs between the values of the open and close position of the first joint angle which i use to open/close the gripper
                 self.Link_two.model.base = pos.T*trotx(-pi/2);
                 self.Link_One.model.base = pos.T*trotx(pi/2);
                 self.Link_One.model.animate([l,deg2rad(25),deg2rad(29)]);   %this animates the grippers joint angles with the l variable to open/close the gripper
                 self.Link_two.model.animate([3*pi/2-l,deg2rad(25),deg2rad(29)]);
%                  self.Link_two.model.teach();
             end
        end


        function MoveGripper(self,gPos)
            qA1 = self.Link_One.model.getpos();   %storing the q values of the first appendage of the gripper in order to animate later in the function
            qA2 = self.Link_two.model.getpos();   %storing the q values of the second appendage of the gripper in order to animate later in the function
            self.Link_One.model.base = gPos.T * trotx(pi/2);
            self.Link_two.model.base = gPos.T *trotz(pi)* trotx(-pi/2);
            self.Link_One.model.animate(qA1);     %animates the first gripper appendage using the q values stored in qA1
            self.Link_two.model.animate(qA2);  
            

            drawnow(); 
       
        end

        function setOffset(self, O)
            
            self.offset(1) = O(1);
            self.offset(2) = O(2);
            self.offset(3) = O(3);


            drawnow(); 
       
        end
    end
end