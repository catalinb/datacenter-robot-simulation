function map = generateMap(width, heigth, widthCol)
map(1:heigth,1:width) = 0;

disp(width/widthCol);

for j=1:(width/widthCol)-1
    if mod(j, 2) == 1
        map(1:heigth, j*widthCol+1:(j+1)*widthCol) = 1;
        
        nrRand = randi(heigth/2) + 1
        
        for i=1:nrRand
            free = randi(heigth);
            map(free, j*widthCol+1:(j+1)*widthCol) = 0;
        end
   end
end