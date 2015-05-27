%ROBOT navigation class
%
% Based on the PRM example by Peter I. Corke
%
% - Probabilistic roadmaps for path planning in high dimensional configuration spaces,
%   L. Kavraki, P. Svestka, J. Latombe, and M. Overmars,
%   IEEE Transactions on Robotics and Automation, vol. 12, pp. 566-580, Aug 1996.
% - Robotics, Vision & Control, Section 5.2.4,
%   P. Corke, Springer 2011.
%
% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
%
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com


% Peter Corke 8/2009.

classdef ROBOT < Navigation

    properties
        npoints         % number of sample points
        distthresh      % distance threshold, links between vertices
                        % must be less than this.

        graph           % graph Object representing random nodes

        vgoal           % index of vertex closest to goal
        vstart          % index of vertex closest to start
        localGoal       % next vertex on the roadmap
        localPath       % set of points along path to next vertex
        gpath           % list of vertices between start and goal
    end

    methods

        % constructor
        function robot = ROBOT(varargin)
            % invoke the superclass constructor, it handles some options
            robot = robot@Navigation(varargin{:});

            % create an empty 2D graph
            robot.graph = PGraph(2);

            % parse out PRM specific options and save in the navigation object
            opt.npoints = 100;
            opt.distthresh = 0.3*max(size(robot.occgrid));
            [opt,args] = tb_optparse(opt, varargin);
            robot.npoints = opt.npoints;
            robot.distthresh = opt.distthresh;
        end

        function s = char(prm)
            % invoke the superclass char() method
            s = char@Navigation(prm);

            % add PRM specific stuff information
            s = char(s, sprintf('  graph size: %d', prm.npoints));
            s = char(s, sprintf('  dist thresh: %f', prm.distthresh));
            s = char(s, char(prm.graph) );
        end


        function plot(prm, varargin)
        %PRM.plot Visualize navigation environment
        %
        % P.plot() displays the occupancy grid with an optional distance field.
        %
        % Options::
        %  'goal'            Superimpose the goal position if set
        %  'nooverlay'       Don't overlay the PRM graph
            clf
        
            unit_size = 100;
            [height, width] = size(prm.occgrid);
            
            rposx = randi(height);
            rposy = randi(width);
            
            while(prm.occgrid(rposx, rposy) == 1)
                rposx = randi(height);
                rposy = randi(width);
            end
            
            ground = imread('images/ground.jpg', 'jpg');
            ground = imresize(ground, [height * unit_size, width * unit_size]);
            
            rack = imread('images/rack.jpg', 'jpg');
            rack = imresize(rack, [unit_size, unit_size]);
            rack = imrotate(rack, 180);
            
            robot = imread('images/robot_loaded.jpg', 'jpg');
            robot = imresize(robot, [unit_size, unit_size]);
            robot = imrotate(robot, 180);
            
            for i = 1:height
                for j = 1:width
                    if(prm.occgrid(i, j) == 1)
                        % TODO: do not display unreachable servers
                        
                        
                        ground(((i - 1) * unit_size + 1):(i * unit_size), ((j - 1) * unit_size + 1):(j * unit_size), :) = rack(1:unit_size, 1:unit_size, :);
                    end
                end
            end
            
            rposx = floor((rposx - 1) * unit_size + 1);
            rposy = floor((rposy - 1) * unit_size + 1);
            
            ground(rposx:(rposx + unit_size - 1), rposy:(rposy + unit_size - 1), :) = robot(1:unit_size, 1:unit_size, :);
            
            imshow(ground);
            
            set(gca, 'Ydir', 'normal');
            xlabel('x');
            ylabel('y');
            hold on;
        end
        
        function n = next(robot, p)
            n = []
        end
        
        function plan(robot)
        end

    end % method
end % classdef
