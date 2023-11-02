classdef Gripper2 < RobotBaseClass

    properties(Access = public)   
        plyFileNameStem = 'Gripper';
    end
    
    methods
%% Constructor
        function self = Gripper2(baseTr)
            self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end           
			self.model.base = self.model.base.T * baseTr;
%             self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

%             self.model.teach();

           
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            link(2) = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            link(3) = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
% 
%             L4 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
%             L5 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
%             L6 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);

            %% Generating models of both appendages of the gripper
            self.model = SerialLink(link, 'name', self.name);
%             self.appendage2 = SerialLink([L4 L5 L6], 'name', 'appendage2');

        end      
    end
end