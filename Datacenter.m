classdef Datacenter
    properties
        map;
        eventmap;
        mapWidth;
        mapHeight;
        mapRackWidth;
        mapRowWidth;
        mapUnitSize;
        mapImage; % caching mapImage background (racks + tiles + landmark radius)
        landmarks;
        landmarkViewRange;
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
            opt.landmarkViewRange = 1.5;
            
            [opt, args] = tb_optparse(opt, varargin);
            
            datacenter.mapWidth = opt.mapWidth;
            datacenter.mapHeight = opt.mapHeight;
            datacenter.mapRackWidth = opt.mapRackWidth;
            datacenter.mapRowWidth = opt.mapRowWidth;
            datacenter.mapUnitSize = opt.mapUnitSize;
            datacenter.landmarkViewRange = opt.landmarkViewRange;
            
            datacenter.map = Datacenter.generateMap(opt.mapWidth, opt.mapHeight, opt.mapRowWidth, opt.mapRackWidth);
            datacenter.landmarks = Datacenter.generateLandmarks(datacenter.map);
            datacenter.eventmap = Datacenter.generateEvents(datacenter.map);
            
            robotPosition = Datacenter.generateRobotPosition(datacenter.map, opt.robotRadius);
            datacenter.robot = Robot(opt.robotRadius, datacenter.landmarks, robotPosition(2), robotPosition(1));
            datacenter.mapImage = datacenter.blitBackground();
        end
        
        function route = generateRobotDiscreteRoute(datacenter, destX, destY)
            route = [];
            
            map = datacenter.map .* -1;

            rPosX = datacenter.robot.estimatedPos(1);
            rPosY = datacenter.robot.estimatedPos(2);
            
            initX = max(floor(rPosX), 1);
            initY = max(floor(rPosY), 1);
            
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

        % TODO: colision behavior.
        function moveRobot(datacenter, destX, destY)
            stepSize = 2 / datacenter.mapUnitSize;

            buf = datacenter.plot(NaN);

            while true
                curPos = datacenter.robot.currentPos;
                newPos = datacenter.robot.stepTowards([destX, destY], stepSize);
                deltaPos = newPos - curPos;
                
                % Determine the possible area of collisions.
                minX = max(floor(min(curPos(1), newPos(1))) - 1, 1);
                minY = max(floor(min(curPos(2), newPos(2))) - 1, 1);

                maxX = min(ceil(max(curPos(1), newPos(1))) + 1, datacenter.mapHeight);
                maxY = min(ceil(max(curPos(2), newPos(2))) + 1, datacenter.mapWidth);
                
                for i = minX : maxX
                        for j = minY : maxY
                            if datacenter.map(i, j) ~= 1
                                continue;
                            end
                            
                            
                        end
                end
                
                % Temporary.
                datacenter.robot.currentPos = newPos;

                remainingDistance = sum(abs([destX, destY] - datacenter.robot.estimatedPos))
                if remainingDistance < 0.1
                    break;
                end

                datacenter.plot(buf);
                pause(0.001);
            end
        end
        
        function solveEventsLocalOptimum(datacenter)
            events = true;
            while(events)
                events = false;
                [heigth, width] = size(datacenter.map);
                min = -1;
                minI = -1;
                minJ = -1;
                solveI = -1;
                solveJ = -1;
                map = datacenter.map();

                for i = 1 : heigth
                    for j = 1 : width
                        if (datacenter.eventmap(i,j) == 1)
                            if (i+1 < heigth && map(i+1, j) == 0)
                                try
                                    route = datacenter.generateRobotContinuousRoute(i + 1, j);

                                    dist = 0;
                                    for k = 2:size(route, 1)
                                        dist = dist + Datacenter.euclideanDistance([route(k-1, 1) route(k-1, 2)], [route(k, 1) route(k, 2)]);
                                    end

                                    if (min == -1 || dist * 2 < min)
                                        min = dist * 2;
                                        minI = i + 1;
                                        minJ = j;
                                        solveI = i;
                                        solveJ = j;
                                        events = true;
                                    end
                                catch causeException
                                end
                            end
                            
                            if (i-1 > 0 && map(i-1, j) == 0)
                                try
                                    route = datacenter.generateRobotContinuousRoute(i - 1, j);

                                    dist = 0;
                                    for k = 2:size(route, 1)
                                        dist = dist + Datacenter.euclideanDistance([route(k-1, 1) route(k-1, 2)], [route(k, 1) route(k, 2)]);
                                    end

                                    if (min == -1 || dist * 2 < min)
                                        min = dist * 2;
                                        minI = i - 1;
                                        minJ = j;
                                        solveI = i;
                                        solveJ = j;
                                        events = true;
                                    end
                                catch causeException
                                end
                            end
                            
                            if (j+1 < width && map(i, j+1) == 0)
                                try
                                    route = datacenter.generateRobotContinuousRoute(i, j+1);

                                    dist = 0;
                                    for k = 2:size(route, 1)
                                        dist = dist + Datacenter.euclideanDistance([route(k-1, 1) route(k-1, 2)], [route(k, 1) route(k, 2)]);
                                    end

                                    if (min == -1 || dist * 2 < min)
                                        min = dist * 2;
                                        minI = i;
                                        minJ = j + 1;
                                        solveI = i;
                                        solveJ = j;
                                        events = true;
                                    end
                                catch causeException
                                end
                            end
                            
                            if (j-1 > 0 && map(i, j-1) == 0)
                                try
                                    route = datacenter.generateRobotContinuousRoute(i, j-1);

                                    dist = 0;
                                    for k = 2:size(route, 1)
                                        dist = dist + Datacenter.euclideanDistance([route(k-1, 1) route(k-1, 2)], [route(k, 1) route(k, 2)]);
                                    end

                                    if (min == -1 || dist * 2 < min)
                                        min = dist * 2;
                                        minI = i;
                                        minJ = j-1;
                                        solveI = i;
                                        solveJ = j;
                                        events = true;
                                    end
                                catch causeException
                                end
                            end
                            
                            
                        else
                            if (datacenter.eventmap(i,j) == 2)
                                events = true;
                                
                                if (i+1 < heigth && map(i+1, j) == 0)
                                    try
                                        route = datacenter.generateRobotContinuousRoute(i + 1, j);

                                        dist = 0;
                                        for k = 2:size(route, 1)
                                            dist = dist + Datacenter.euclideanDistance([route(k-1, 1) route(k-1, 2)], [route(k, 1) route(k, 2)]);
                                        end

                                        if (min == -1 || dist< min)
                                            min = dist;
                                            minI = i + 1;
                                            minJ = j;
                                            solveI = i;
                                            solveJ = j;
                                            events = true;
                                        end
                                    catch causeException
                                    end
                                end

                                if (i-1 > 0 && map(i-1, j) == 0)
                                    try
                                        route = datacenter.generateRobotContinuousRoute(i - 1, j);

                                        dist = 0;
                                        for k = 2:size(route, 1)
                                            dist = dist + Datacenter.euclideanDistance([route(k-1, 1) route(k-1, 2)], [route(k, 1) route(k, 2)]);
                                        end

                                        if (min == -1 || dist< min)
                                            min = dist;
                                            minI = i - 1;
                                            minJ = j;
                                            solveI = i;
                                            solveJ = j;
                                            events = true;
                                        end
                                    catch causeException
                                    end
                                end

                                if (j+1 < width && map(i, j+1) == 0)
                                    try
                                        route = datacenter.generateRobotContinuousRoute(i, j+1);

                                        dist = 0;
                                        for k = 2:size(route, 1)
                                            dist = dist + Datacenter.euclideanDistance([route(k-1, 1) route(k-1, 2)], [route(k, 1) route(k, 2)]);
                                        end

                                        if (min == -1 || dist< min)
                                            min = dist;
                                            minI = i;
                                            minJ = j + 1;
                                            solveI = i;
                                            solveJ = j;
                                            events = true;
                                        end
                                    catch causeException
                                    end
                                end

                                if (j-1 > 0 && map(i, j-1) == 0)
                                    try
                                        route = datacenter.generateRobotContinuousRoute(i, j-1);

                                        dist = 0;
                                        for k = 2:size(route, 1)
                                            dist = dist + Datacenter.euclideanDistance([route(k-1, 1) route(k-1, 2)], [route(k, 1) route(k, 2)]);
                                        end

                                        if (min == -1 || dist< min)
                                            min = dist;
                                            minI = i;
                                            minJ = j-1;
                                            solveI = i;
                                            solveJ = j;
                                            events = true;
                                        end
                                    catch causeException
                                    end
                                end
                                
                            end
                        end
                    end
                end
                
                if (events)
                    datacenter.routeRobot(minI, minJ);
                    type =  datacenter.eventmap(solveI, solveJ);
                    datacenter.eventmap(solveI, solveJ) = 0;
                    datacenter.mapImage = datacenter.blitBackground();
                    pause(0.5*type);
                end
                
            end
        end

        function mapImage = blitBackground(datacenter)
             % Don't want the image size warning showing up.
            warning('off', 'Images:initSize:adjustingMag');

            mapImageHeight = datacenter.mapHeight * datacenter.mapUnitSize;
            mapImageWidth = datacenter.mapWidth * datacenter.mapUnitSize;

            % Read, resize and rotate used images.
            mapImage = imread('images/ground.jpg', 'jpg');
            mapImage = imresize(mapImage, [mapImageHeight, mapImageWidth]);

            rackSize = datacenter.mapUnitSize;

            rack0 = imread('images/rack.jpg', 'jpg');
            rack0 = imresize(rack0, [rackSize, rackSize]);
            rack0 = imrotate(rack0, 180);
            
            rack1 = imread('images/rack_warning.jpg', 'jpg');
            rack1 = imresize(rack1, [rackSize, rackSize]);
            rack1 = imrotate(rack1, 180);
            
            rack2 = imread('images/rack_fault.jpg', 'jpg');
            rack2 = imresize(rack2, [rackSize, rackSize]);
            rack2 = imrotate(rack2, 180);

            % Draw racks.
            for i = 1 : datacenter.mapHeight
                for j = 1 : datacenter.mapWidth
                    if(datacenter.map(i, j) == 1)
                        if (datacenter.eventmap(i,j)==0)
                            mapImage(((i - 1) * datacenter.mapUnitSize + 1) : (i * datacenter.mapUnitSize), ((j - 1) * datacenter.mapUnitSize + 1) : (j * datacenter.mapUnitSize), :) = rack0(:, :, :);
                        else
                            if (datacenter.eventmap(i,j)==1)
                                mapImage(((i - 1) * datacenter.mapUnitSize + 1) : (i * datacenter.mapUnitSize), ((j - 1) * datacenter.mapUnitSize + 1) : (j * datacenter.mapUnitSize), :) = rack1(:, :, :);
                            else
                                mapImage(((i - 1) * datacenter.mapUnitSize + 1) : (i * datacenter.mapUnitSize), ((j - 1) * datacenter.mapUnitSize + 1) : (j * datacenter.mapUnitSize), :) = rack2(:, :, :);
                            end
                        end    
                    end
                end
            end

            % Draw landmarks view ranges.
            shapeInserter = vision.ShapeInserter('Shape', 'Circles', 'BorderColor', 'Custom', 'CustomBorderColor', uint8([255 255 0]));
            circles = [];

            for i = 1 : size(datacenter.landmarks, 1)
                circles = [circles; [fliplr(datacenter.landmarks(i, :)) datacenter.landmarkViewRange] .* datacenter.mapUnitSize];
            end

            mapImage = step(shapeInserter, mapImage, uint32(circles));
        end

        function buf = plot(datacenter, buffer)
            mapImage = datacenter.mapImage();

            robotSize = ceil(datacenter.mapUnitSize * datacenter.robot.radius);
            robot = imread('images/robot.jpg', 'jpg');
            robot = imresize(robot, [robotSize, robotSize]);

            % Draw robot.
            posX = datacenter.robot.currentPos(1);
            posY = datacenter.robot.currentPos(2);
            rposx = max(floor((posX - datacenter.robot.radius) * datacenter.mapUnitSize), 1);
            rposy = max(floor((posY - datacenter.robot.radius) * datacenter.mapUnitSize), 1);

            mapImage(rposx : (rposx + robotSize - 1), rposy : (rposy + robotSize - 1), :) = robot(:, :, :);

            particleShape = vision.ShapeInserter('Shape', 'Circles', 'BorderColor', 'Custom', 'CustomBorderColor', uint8([0 255 255]));
            particles = [];
            
            for i = 1 : size(datacenter.robot.particles, 1)
                particles = [particles; [fliplr(datacenter.robot.particles(i, :)) 0.5] .* datacenter.mapUnitSize];
            end
            
            mapImage = step(particleShape, mapImage, uint32(particles));
            
            if (~isa(buffer, 'matlab.graphics.primitive.Image'))
                % Set axis.
                set(gca, 'Ydir', 'normal');
                xlabel('x');
                ylabel('y');
                hold on;

                % Do not close the current window.
                clf('reset');
                buf = imshow(mapImage, 'Border', 'tight', 'InitialMagnification', 100);
            else
                set(buffer, 'CData', mapImage);
            end
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
        
        function landmarks = generateLandmarks(map)
            landmarks = [];
            [height, width] = size(map);
            
            for i = 1 : height
                for j = 1 : width
                    if map(i, j) ~= 1
                        continue;
                    end
                    
                    if i ~= 1 && i ~= height && and(map(i - 1, j), map(i + 1, j))
                        continue;
                    end
                    
                    if j ~= 1 && j ~= width && and(map(i, j - 1), map(i, j + 1))
                        continue;
                    end
                    
                    landmarks = [landmarks; [i j] - 0.5];
                end
            end
        end
        
        function eventmap = generateEvents(map)
            events = [];
            [heigth, width] = size(map);
            eventmap(1 : heigth, 1 : width) = 0;
            
            for i=1: heigth
                for j=1:width
                    if (map(i, j)) ~= 1
                        continue;
                    end
                    
                    event = randi(100);
                    
                    if (event < 10)
                        eventmap(i, j) = 2;
                    else
                        if (event < 25)
                            eventmap(i, j)=1;
                        else
                            eventmap(i, j)=0;
                        end
                    end
                    
                end
            end
        end
        
        function position = generateRobotPosition(map, robotRadius)
            [height, width] = size(map);
            
            rposy = randi(height);
            rposx = ceil(robotRadius);
            
            while map(rposx, rposy) == 1 && rposy < robotRadius && rposy + robotRadius > height
                rposy = randi(height);
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
