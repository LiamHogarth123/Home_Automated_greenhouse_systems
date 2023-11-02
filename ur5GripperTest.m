classdef ur5GripperTest < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'UR3';
    end
    
    methods
%% Constructor
        function self = ur5GripperTest(baseTr,useTool,toolFilename)
            if nargin < 3
                
                if nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end            
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
%             self.PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)
              %% Defining DH parameters of the gripper links
            L1 = Link('d', 0, 'a', 0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            L2 = Link('d', 0, 'a', 0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            L3 = Link('d', 0, 'a', 0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);

            L4 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            L5 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            L6 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);

            %% Generating models of both appendages of the gripper
            self.model = SerialLink([L1 L2 L3], 'name', 'appendage1');
%             self.appendage2 = SerialLink([L4 L5 L6], 'name', 'appendage2');
% 
%             % self.appendage1.teach();
%             % hold on
%             % self.appendage2.teach();
% 
            workspace = [-0.2 0.2 -0.2 0.2 -0.2 0.2];
            scale = 1;
            q = [deg2rad(40), deg2rad(25), deg2rad(29)];

            %q = zeros(1, 3);

            self.model.plot(q,'workspace',workspace,'scale',scale);
%             hold on;
%             self.appendage2.plot(q,'workspace',workspace,'scale',scale, 'notiles', 'noshadow', 'noname', 'nobase', 'nowrist');
% 
%            
    

%             link(1) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%             link(2) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
%             link(3) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
%             link(4) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
%             link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
%             link(6) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
%              
%             self.model = SerialLink(link,'name',self.name);
        end      
    end
end