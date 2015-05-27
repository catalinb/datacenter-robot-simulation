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
        
        function moveRobot(datacenter, destX, destY)
            while (abs(datacenter.robot.posX-destX) > 0.001 || abs(datacenter.robot.posY-destY) > 0.001)
                disp(datacenter.robot.posX);
                if (datacenter.robot.posX > destX)
                    datacenter.robot.posX = datacenter.robot.posX - 0.5;
                else if (datacenter.robot.posX < destX)
                        datacenter.robot.posX = datacenter.robot.posX + 0.5;
                    else if (datacenter.robot.posY > destY)
                            datacenter.robot.posY = datacenter.robot.posY - 0.5;
                        else if (datacenter.robot.posY < destY)
                                datacenter.robot.posY = datacenter.robot.posY + 0.5;
                            end
                        end
                    end
                end
                datacenter.plot()
                pause(0.001)
            end
        end

        function moveRoute(datacenter, destX, destY)
            aux(1:datacenter.mapHeight, 1:datacenter.mapWidth) = 0;

            aux(floor(datacenter.robot.posX), floor(datacenter.robot.posY)) = 1;
            
            st = 1;
            fn = 1;
            queueX(st) = floor(datacenter.robot.posX);
            queueY(st) = floor(datacenter.robot.posY);
            
            while(true)
                if (queueX(st) == destX && queueY(st) == destY || st > fn)
                    break;
                end
                
                if (queueX(st)+1 < datacenter.mapHeight+1 && datacenter.map(queueX(st)+1, queueY(st)) == 0 && aux(queueX(st)+1, queueY(st)) == 0)
                    fn=fn+1;
                    aux(queueX(st)+1, queueY(st)) = aux(queueX(st), queueY(st))+1;
                    queueX(fn)=queueX(st)+1;
                    queueY(fn)=queueY(st);
                end
                
                if (queueX(st)-1 > 0 && datacenter.map(queueX(st)-1, queueY(st)) == 0 && aux(queueX(st)-1, queueY(st)) == 0)
                    fn=fn+1;
                    aux(queueX(st)-1, queueY(st)) = aux(queueX(st), queueY(st))+1;
                    queueX(fn)=queueX(st)-1;
                    queueY(fn)=queueY(st);
                end
                
                if (queueY(st)+1 < datacenter.mapWidth+1 && datacenter.map(queueX(st), queueY(st)+1) == 0 && aux(queueX(st), queueY(st)+1) == 0)
                    fn=fn+1;
                    aux(queueX(st), queueY(st)+1) = aux(queueX(st), queueY(st))+1;
                    queueX(fn)=queueX(st);
                    queueY(fn)=queueY(st)+1;
                end

                if (queueY(st)-1 > 0 && datacenter.map(queueX(st), queueY(st)-1) == 0 && aux(queueX(st), queueY(st)-1) == 0)
                    fn=fn+1;
                    aux(queueX(st), queueY(st)-1) = aux(queueX(st), queueY(st))+1;
                    queueX(fn)=queueX(st);
                    queueY(fn)=queueY(st)-1;
                end
                
                st = st+1;
            end
            
            st = 1;
            movesX(st) = destX;
            movesY(st) = destY;
            
            while(true)
                if (movesX(st) == floor(datacenter.robot.posX) && movesY(st) == floor(datacenter.robot.posY))
                    break;
                end
                
                if (movesX(st)+1 < datacenter.mapHeight+1 && aux(movesX(st)+1, movesY(st)) == aux(movesX(st), movesY(st)) - 1)
                    movesX(st+1) = movesX(st)+1;
                    movesY(st+1) = movesY(st);
                    st=st+1;
                else
                    if (movesX(st)-1 > 0 && aux(movesX(st)-1, movesY(st)) == aux(movesX(st), movesY(st)) - 1)
                        movesX(st+1) = movesX(st)-1;
                        movesY(st+1) = movesY(st);
                        st=st+1;
                    else
                        if (movesY(st)+1 < datacenter.mapWidth+1 && aux(movesX(st), movesY(st)+1) == aux(movesX(st), movesY(st)) - 1)
                            movesX(st+1) = movesX(st);
                            movesY(st+1) = movesY(st)+1;
                            st=st+1;
                        else
                            if (movesY(st)-1 > 0 && aux(movesX(st), movesY(st)-1) == aux(movesX(st), movesY(st)) - 1)
                                movesX(st+1) = movesX(st);
                                movesY(st+1) = movesY(st)-1;
                                st=st+1;
                            end
                        end
                    end
                end
            end
                        
            while(st > 0)
                disp(st)
                datacenter.moveRobot(movesX(st), movesY(st))
                st = st - 1;
            end
            
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