close all;
load('Release/tree.txt');
% map = imread('../ompl/tests/resources/ppm/empty.ppm');
map = imread('../ompl/tests/resources/ppm/floor.ppm');
figure(1)
imshow(map);
hold on;
scatter(tree(:,1), tree(:,2), 5, tree(:,5));

for i = 1:size(tree,1)
    line([tree(i,1), tree(i,3)], [tree(i,2), tree(i,4)], 'Color', 'b')
end

load('Release/tree2.txt');
% map = imread('../ompl/tests/resources/ppm/empty.ppm');
map = imread('../ompl/tests/resources/ppm/floor.ppm');
figure(2)
imshow(map);
hold on;
scatter(tree2(:,1), tree2(:,2), 5, tree2(:,5));

for i = 1:size(tree2,1)
    line([tree2(i,1), tree2(i,3)], [tree2(i,2), tree2(i,4)], 'Color', 'b')
end