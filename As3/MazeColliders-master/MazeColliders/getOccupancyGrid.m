image = imread('map4_2.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
grid = binaryOccupancyMap(bwimage);
grid2 = binaryOccupancyMap(grid, 4);
inflate(grid2,0.25)
figure(3)
show(grid2)
figure(4)
grid3 = binaryOccupancyMap(grid2, 1);
show(grid3)
mat = occupancyMatrix(grid)