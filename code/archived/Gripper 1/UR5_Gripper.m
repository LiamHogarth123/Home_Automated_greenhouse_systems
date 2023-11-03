classdef UR5_Gripper < handle
    properties
        appendage1; %added ;
        appendage2;
        %plyFileNameStem = '';
    end

    methods
        function self = UR5_Gripper()
            self.Grippy();
            
            
            % if nargin < 1 
            %     base = eye(3);
            % end
        end
    end
    methods
        function Grippy(self)

            robot = LinearUR5;
            hold on

            %% Defining DH parameters of the gripper links
            L1 = Link('d', 0, 'a', 0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            L2 = Link('d', 0, 'a', 0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            L3 = Link('d', 0, 'a', 0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);

            L4 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            L5 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);
            L6 = Link('d', 0, 'a', -0.05, 'alpha', 0, 'offset', 0, 'qlim', [-pi, pi]);

            %% Generating models of both appendages of the gripper
            self.appendage1 = SerialLink([L1 L2 L3], 'name', 'appendage1');
            self.appendage2 = SerialLink([L4 L5 L6], 'name', 'appendage2');

            % self.appendage1.teach();
            % hold on
            % self.appendage2.teach();

            workspace = [-0.2 0.2 -0.2 0.2 -0.2 0.2];
            scale = 1;
            q = [deg2rad(40), deg2rad(25), deg2rad(29)];

            %q = zeros(1, 3);

            self.appendage1.plot(q,'workspace',workspace,'scale',scale, 'notiles', 'noshadow', 'noname', 'nobase', 'nowrist');
            hold on;
            self.appendage2.plot(q,'workspace',workspace,'scale',scale, 'notiles', 'noshadow', 'noname', 'nobase', 'nowrist');

           
        end

    end

end



     