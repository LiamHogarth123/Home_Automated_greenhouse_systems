classdef SpawnLinearUR5
    properties
        
        
    end

    methods

        function spawnLinearUR5(q)
            global r;
            % q = [0 0 0 0 0 0 0];
            r = LinearUR5();
            r.model;
            % r.model.teach(q)
        
        end

        function moveRobot(q)
            r.model.animate(q);
            drawnow();
        end

    end
end








 
% function spawnLinearUR5()
%     global r;
%     % q = [0 0 0 0 0 0 0];
%     r = LinearUR5();
%     r.model;
%     % r.model.teach(q)
% end
