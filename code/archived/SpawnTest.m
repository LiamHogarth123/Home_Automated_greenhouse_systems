classdef SpawnTest

    properties
        r;
        Q;
    end

    methods
        function obj = spawnTest(q)
            obj.Q = q;
            obj.r = LinearUR5();
            obj.r.model;
            obj.r.model.teach(q)
        end

    end
end