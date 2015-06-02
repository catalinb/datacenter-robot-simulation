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
        
        function route = generateRobotDiscreteRoute(datacenter, destX, destY)
            route = [];
            
            map = datacenter.map .* -1;
            
            initX = floor(datacenter.robot.posX);
            initY = floor(datacenter.robot.posY);
            
            assert(map(initX, initY) == 0);
            
            if initX == destX && initY == destY
                return;
            end
            
            % [curX curY dist prevX prevY]
            queue = [[initX initY 0 -1 -1];];

            i = 1;
            
            while i <= size(queue, 1)
                posX = queue(i, 1);
                posY = queue(i, 2);
                posD = queue(i, 3);
            
                if posX == destX && posY == destY
                    % truncate the queue.
                    queue = queue(1 : i, :);
                    
                    break;
                end
                
                if posX > 1 && map(posX - 1, posY) == 0 && sum([posX - 1 posY] == [initX initY]) < 2
                    queue = [queue; [posX - 1 posY posD + 1 posX posY]];
                    map(posX - 1, posY) = posD + 1;
                end
                
                if posX < datacenter.mapHeight && map(posX + 1, posY) == 0  && sum([posX + 1 posY] == [initX initY]) < 2
                    queue = [queue; [posX + 1 posY posD + 1 posX posY]];
                    map(posX + 1, posY) = posD + 1;
                end
                
                if posY > 1 && map(posX, posY - 1) == 0 && sum([posX posY - 1] == [initX initY]) < 2
                    queue = [queue; [posX posY - 1 posD + 1 posX posY]];
                    map(posX, posY - 1) = posD + 1;
                end
                
                if posY < datacenter.mapWidth && map(posX, posY + 1) == 0 && sum([posX posY + 1] == [initX initY]) < 2 
                    queue = [queue; [posX posY + 1 posD + 1 posX posY]];
                    map(posX, posY + 1) = posD + 1;
                end
                
                i = i + 1;
            end
            
            prevX = queue(size(queue, 1), 1);
            prevY = queue(size(queue, 1), 2);
            
            % Destination has been reached.
            assert(prevX == destX && prevY == destY);
            
            for i = size(queue, 1) : -1 : 1
                posX = queue(i, 1);
                posY = queue(i, 2);
                
                if posX ~= prevX || posY ~= prevY
                    continue;
                end
                
                route = [[posX posY]; route];
                
                prevX = queue(i, 4);
                prevY = queue(i, 5);
            end
        end
        
        function route = generateRobotContinuousRoute(datacenter, destX, destY)
            route = [];
            
            droute = datacenter.generateRobotDiscreteRoute(destX, destY);
            
            old = zeros(size(droute, 1), 1);
            old(1) = 1;
            
            oldRoutes = zeros(size(droute, 1), size(droute, 1));
            oldRoutes(1, 1) = 1;
            
            oldDistances = zeros(size(droute, 1), 1);
            
            routes = zeros(size(droute, 1), size(droute, 1));
            routes(1, 1) = 1;
            
            for i = 1 : size(droute, 1) - 1
                curX = droute(i, 1);
                curY = droute(i, 2);
                
                new = zeros(size(droute, 1), 1);
                newRoutes = zeros(size(droute, 1), size(droute, 1));
                newDistances = zeros(size(droute, 1), 1);
                
                for j = i + 1 : size(droute, 1)
                    posX = droute(j, 1);
                    posY = droute(j, 2);
                    
                    % Distance from current position to possible next position.
                    dcp =  Datacenter.euclideanDistance([curX curY], [posX posY]);
                    
                    % Determine the possible area of collisions.
                    minX = min(curX, posX);
                    minY = min(curY, posY);

                    maxX = max(curX, posX);
                    maxY = max(curY, posY);
                    
                    % Determine if there are any collisions.
                    collides = false;

                    for k = minX : maxX
                        for l = minY : maxY
                            if datacenter.map(k, l) ~= 1
                                continue;
                            end

                            % Vertices of the server. Used to determine
                            % collision.
                            vertices = [[k l]; [k + 1 l]; [k l + 1]; [k + 1 l + 1]];
                            for m = 1 : size(vertices, 1)
                                verX = vertices(m, 1);
                                verY = vertices(m, 2);

                                % Distance from current position to vertice.
                                dcv = Datacenter.euclideanDistance([curX curY], [verX verY]);
                                % Distance from possible next position to vertice.
                                dpv = Datacenter.euclideanDistance([posX posY], [verX verY]);
                                
                                % Semiperimeter and area.
                                p = (dcp + dcv + dpv) / 2;
                                a = sqrt(p * (p - dcp) * (p - dcv) * (p - dpv));

                                % Distance from vertice to direct route.
                                d = (2 * a) / dcp;

                                % The robot colides if the distance is lower
                                % than its radius.
                                if d < datacenter.robot.radius
                                    collides = true;
                                    break;
                                end
                            end

                            if collides == true
                                break;
                            end
                        end

                        if collides == true
                            break;
                        end
                    end
                    
                    if collides == false
                        new(j) = old(i) + 1;
                        
                        newRoutes(j, :) = oldRoutes(i, :);
                        newRoutes(j, new(j)) = j;
                        
                        newDistances(j) = oldDistances(i) + Datacenter.euclideanDistance(droute(newRoutes(j, old(i)), :), droute(newRoutes(j, new(j)), :));
                        
                        if old(j) ~= 0 && oldDistances(j) < newDistances(j)
                            new(j) = old(j);
                            newRoutes(j, :) = oldRoutes(j, :);
                            newDistances(j) = oldDistances(j);
                        end
                    end
                end
               
                old = new;
                oldRoutes = newRoutes;
                oldDistances = newDistances;
            end
            
            routeIndices = nonzeros(oldRoutes(size(oldRoutes, 1), :));
            route = droute(routeIndices, :);
        end
        
        function routeRobot(datacenter, destX, destY)
            route = datacenter.generateRobotContinuousRoute(destX, destY);
            route = route(2 : length(route), :);
            
            for i = 1 : size(route, 1)
                nextX = route(i, 1);
                nextY = route(i, 2);
                
                datacenter.moveRobot(nextX, nextY);
            end
        end
        
        function moveRobot(datacenter, destX, destY)
            dx = destX - datacenter.robot.posX;
            dy = destY - datacenter.robot.posY;
            
            d =  sqrt(dx ^ 2 + dy ^ 2);
            
            noSteps = floor(datacenter.mapUnitSize / 5 * d);
            stepSize = d / noSteps;
            
            sx = (stepSize * dx) / d;
            sy = (stepSize * dy) / d;
            
            for i = 1 : noSteps
                disp([i noSteps dx dy stepSize sx sy]);
                if i == noSteps
                    datacenter.robot.posX = destX;
                    datacenter.robot.posY = destY;
                else
                    datacenter.robot.posX = datacenter.robot.posX + sx;
                    datacenter.robot.posY = datacenter.robot.posY + sy;
                end
                
                datacenter.plot();
                pause(0.001);
            end
        end
        
        function plot(datacenter, varargin)
            clf;
            
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
        
        function d = euclideanDistance(p1, p2)
            x1 = p1(1);
            y1 = p1(2);
            
            x2 = p2(1);
            y2 = p2(2);
            
            d = sqrt((x1 - x2) ^ 2 + (y1 - y2) ^ 2);
        end
    end
end