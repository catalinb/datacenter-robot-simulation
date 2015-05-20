function [x, y] = generateRobotPosition(map)
    [n,m] = size(map);
    
    x = randi(n);
    y = randi(m);
    while (map(x,y) == 1)
        x = randi(n);
        y = randi(m);
    end
end