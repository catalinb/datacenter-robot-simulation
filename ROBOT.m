classdef Robot < Navigation
    properties
        npoints;
        distthresh;
        graph;
        vgoal;
        vstart;
        localGoal;
        localPath;
        gpath;
        posX;
        posY;
        radius;
    end

    methods
        function robot = Robot(radius, posX, posY, varargin)
            robot = robot@Navigation(varargin{:});

            robot.graph = PGraph(2);
            robot.radius = radius;
            robot.posX = posX;
            robot.posY = posY;
            
            opt.npoints = 100;
            opt.distthresh = 0.3 * max(size(robot.occgrid));
            
            [opt, args] = tb_optparse(opt, varargin);
            
            robot.npoints = opt.npoints;
            robot.distthresh = opt.distthresh;
        end

        function s = char(prm)
            s = char@Navigation(prm);

            s = char(s, sprintf('  graph size: %d', prm.npoints));
            s = char(s, sprintf('  dist thresh: %f', prm.distthresh));
            s = char(s, char(prm.graph) );
        end
        
        function n = next(prm, p)
            n = []
        end
        
        function plan(prm)
        end
    end
end
