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
            % link(1) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0);
            % link(2) = Link('d',0,'a',2,'alpha',0,'qlim',[-pi pi],'offset',0);
            % link(3) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',0);
            % link(4) = Link('d',0,'a',1,'alpha',-pi/2,'qlim',[-pi pi],'offset',0);
            % link(5) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0);
            % link(6) = Link('d',1,'a',0.5,'alpha',0,'qlim',[-pi pi],'offset',0);

            % link(1) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
            % link(2) = Link('d',0,'a',0.409,'alpha',0,'qlim',[-pi pi],'offset',pi/2)
            % link(3) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',pi/2)
            % link(4) = Link('d',0.367,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
            % link(5) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',pi)
            % link(6) = Link('d',0.127,'a',0,'alpha',0,'qlim',[-pi pi],'offset',0)

            link(1) = Link('d',0.155,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
            link(2) = Link('d',0,'a',0.409,'alpha',0,'qlim',[-pi pi],'offset',pi/2)
            link(3) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-160 160]*pi/180,'offset',pi/2)
            link(4) = Link('d',0.367,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',0)
            link(5) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi],'offset',pi)
            link(6) = Link('d',0.127,'a',0,'alpha',0,'qlim',[-pi pi],'offset',0)
      

            % UR3e
            % link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            % link(2) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            % link(3) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            % link(4) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            % link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            % link(6) = Link('d',	0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            % Incorporate joint limits
           
            % link(1).qlim = [-360 360]*pi/180;
            % link(2).qlim = [-360 360]*pi/180;
            % link(3).qlim = [-160 160]*pi/180;
            % link(4).qlim = [-360 360]*pi/180;
            % link(5).qlim = [-360 360]*pi/180;
            % link(6).qlim = [-360 360]*pi/180;
        
            % link(4).offset = -pi/2;
            % link(5).offset = -pi/2;
            
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end