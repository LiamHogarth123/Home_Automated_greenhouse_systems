classdef A0509 < RobotBaseClass
%% A0509
    properties(Access = public)              
        plyFileNameStem = 'A0509';
    end
    
    methods
%% Define robot Function 
        function self = A0509(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            %self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)
            link(1) = Link('d',0.155,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0);
            link(2) = Link('d',0,'a',0.409,'alpha',0,'qlim',[-pi pi],'offset',pi/2);
            link(3) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',pi/2);
            link(4) = Link('d',0.367,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0);
            link(5) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',pi);
            link(6) = Link('d',0.127,'a',0,'alpha',0,'qlim',[-pi pi],'offset',0);

            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end