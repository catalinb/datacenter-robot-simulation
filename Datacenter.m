classdef Datacenter
    properties
        map;
        mapWidth;
        mapHeight;
        mapRackWidth;
        mapRowWidth;
        mapUnitSize;
        robot;
        robotRadius;
    end
    
    methods
        function datacenter = Datacenter(varargin)
            opt.mapWidth = 30;
            opt.mapHeight = 30;
            opt.mapRackWidth = 3;
            opt.mapRowWidth = 2;
            opt.mapUnitSize = 100;
            opt.robotRadius = 1;
            
            [opt, args] = tb_optparse(opt, varargin);
            
            datacenter.mapWidth = opt.mapWidth;
            datacenter.mapHeight = opt.mapHeight;
            datacenter.mapRackWidth = opt.mapRackWidth;
            datacenter.mapRowWidth = opt.mapRowWidth;
            datacenter.mapUnitSize = opt.mapUnitSize;
            
            datacenter.map = Datacenter.generateMap(opt.mapWidth, opt.mapHeight, opt.mapRowWidth, opt.mapRackWidth);
            
            robotPosition = Datacenter.generateRobotPosition(datacenter.map, opt.robotRadius);
            datacenter.robot = Robot(opt.robotRadius, robotPosition(2), robotPosition(1));
        end
        
        function plot(datacenter, varargin)
            clf
            
            groundHeight = datacenter.mapHeight * datacenter.mapUnitSize;
            groundWidth = datacenter.mapWidth * datacenter.mapUnitSize;
            
            ground = imread('images/ground.jpg', 'jpg');
            ground = imresize(ground, [groundHeight, groundWidth]);
            
            rackSize = datacenter.mapUnitSize;
            
            rack = imread('images/rack.jpg', 'jpg');
            rack = imresize(rack, [rackSize, rackSize]);
            rack = imrotate(rack, 180);
            
            robotSize = ceil(datacenter.mapUnitSize * datacenter.robot.radius);
            
            robot = imread('images/robot.jpg', 'jpg');
            robot = imresize(robot, [robotSize, robotSize]);
            
            for i = 1 : datacenter.mapHeight
                for j = 1 : datacenter.mapWidth
                    if(datacenter.map(i, j) == 1)
                        ground(((i - 1) * datacenter.mapUnitSize + 1) : (i * datacenter.mapUnitSize), ((j - 1) * datacenter.mapUnitSize+ 1) : (j * datacenter.mapUnitSize), :) = rack(:, :, :);
                    end
                end
            end
            
            rposx = max(floor((datacenter.robot.posX - datacenter.robot.radius) * datacenter.mapUnitSize), 1);
            rposy = max(floor((datacenter.robot.posY - datacenter.robot.radius) * datacenter.mapUnitSize), 1);
            
            ground(rposx : (rposx + robotSize - 1), rposy : (rposy + robotSize - 1), :) = robot(:, :, :);
            
            imshow(ground);
            
            set(gca, 'Ydir', 'normal');
            xlabel('x');
            ylabel('y');
            hold on;
        end
    end
    
    methods(Static)
        function map = generateMap(width, height, freeRowWidth, occupiedRowWidth)
            map(1 : height, 1 : width) = 0;
            
            for i = freeRowWidth + 1 : freeRowWidth + occupiedRowWidth : width
                map(1 : height, i) = 1;
                if i + occupiedRowWidth - 1 < width
                    map(1 : height, i + occupiedRowWidth - 1) = 1;
                end
                
                throughRackRowsNo = max(randi(max(floor(height / freeRowWidth / 2), 1)), 1);
                for j = 1 : throughRackRowsNo
                    freeThroughRackRow = randi(height);
                    while freeThroughRackRow + freeRowWidth >= height
                        freeThroughRackRow = randi(height);
                    end
 
                    if freeThroughRackRow > 1 && nnz(map(freeThroughRackRow - 1, i : min(i + occupiedRowWidth - 1, width)) == 1) > 0
                        map(freeThroughRackRow - 1, i : min(i + occupiedRowWidth - 1, width)) = 1;
                    end
                    if freeThroughRackRow + freeRowWidth + 1 <= height && nnz(map(freeThroughRackRow + freeRowWidth + 1, i : min(i + occupiedRowWidth - 1, width)) == 1) > 0
                        map(freeThroughRackRow + freeRowWidth + 1, i : min(i + occupiedRowWidth - 1, width)) = 1;
                    end
                    
                    map(freeThroughRackRow : min(freeThroughRackRow + freeRowWidth, height), i : min(i + occupiedRowWidth - 1, width)) = 0;
                end
            end
        end
        
        function position = generateRobotPosition(map, robotRadius)
            [height, width] = size(map);
            
            rposy = randi(width);
            rposx = ceil(robotRadius);
            
            while map(rposx, rposy) == 1 && rposy < robotRadius && rposy + robotRadius > width
                rposy = randi(width);
            end
            
            position = [rposx, rposy];
        end
    end
end