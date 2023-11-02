classdef GripperRight < RobotBaseClass

    properties(Access = public)   
        plyFileNameStem = 'GripperRight';
    end
    
    methods
%% Constructor
        function self = GripperRight(baseTr)
            self.CreateModel();
            if nargin < 1			
				baseTr = [1,0,0,-0.025; 0,1,0,0; 0,0,1,0; 0,0,0,1];				
            end           
			self.model.base = self.model.base.T * baseTr;
%             self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

%             self.model.teach();

           
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d', 0, 'a', 0.075, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            link(2) = Link('d', 0, 'a', 0.075, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            link(3) = Link('d', 0, 'a', 0.075, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
% 
%             L4 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
%             L5 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
%             L6 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
q = [deg2rad(40), deg2rad(25), deg2rad(29)];
            %% Generating models of both appendages of the gripper
            self.model = SerialLink(link, 'name', self.name);
%             self.appendage2 = SerialLink([L4 L5 L6], 'name', 'appendage2');

        end      
    end
end