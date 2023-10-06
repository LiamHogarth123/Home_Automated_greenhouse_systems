classdef Lab2AssessmentTest1 < handle
%#ok<*NASGU>
%#ok<*NOPRT>
%#ok<*TRYNC>

    properties 
           
    end

    methods 
		function self = Lab2AssessmentTest1()
			clf
			clc
			input('Press enter to begin')
% 			self.LinearUr5();
% 			self.A0509();
% 			self.Environmnet();
% 			self.thread1();
		end
	end

    methods(Static)
%% Question 1: Animate transform (Quad copter flying)
    function LinearUr5()
            mdl_LinearUr5

            workspace = [-1 2.5 -2 2 -1 2];
            
            qleft =  [5*pi/3,0,0,0,8*pi/2,0,0];
            qright = [7*pi/4,0,0,0,0,0,-3*pi/20];
            
            scale = 0.5;
            
 
        end

%% Question 2: Plotting and moving the herd of RobotCows
        function A0509()
             mdl_A0509
            
            % specify workspace
            workspace = [-1 2.5 -2 2 -1 2];
            
            scale = 0.5;
            
            qz = [0,0,0,0,0,0];
            
            p560.plot(qz,'workspace',workspace,'scale',scale); 
            

            hold on;
                
        end

%% Question3point8        
function Environmnet()  
            
        end

%% Question 4 Derive the DH parameters for the simple 3 link manipulator provided. 


        function thread1()

            
            

        end
    end
end