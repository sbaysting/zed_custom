% Load PCD files and put them into XYZ format

pc1full = readPcd('sakib1.pcd');
pc1 = [pc1full(:,1),pc1full(:,2),pc1full(:,3)];
pc1color = pc1full(:,4);
pc1color = unpackRGBFloat(single(pc1color));
pc2full = readPcd('sakib2.pcd');
pc2 = [pc2full(:,1),pc2full(:,2),pc2full(:,3)];
pc2color = pc2full(:,4);
pc2color = unpackRGBFloat(single(pc2color));
clear pc1full
clear pc2full

% Convert to point cloud
pc1 = pointCloud(pc1,'Color',pc1color);
pc2 = pointCloud(pc2,'Color',pc2color);

% Downsample with a box grid filter and set the size of grid filter to 
% be 10cm. The grid filter divides the point cloud space into cubes. 
% Points within each cube are combined into a single output point by 
% averaging their X,Y,Z coordinates

gridSize = 0.1;
fixed = pcdownsample(pc1, 'gridAverage', gridSize);
moving = pcdownsample(pc2, 'gridAverage', gridSize);

% Use the ICP algorithm to estimate the 3-D rigid transformation on 
% the downsampled data

tform = pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
ptCloudAligned = pctransform(pc2,tform);

% Create the world scene with the registered data

mergeSize = 0.015;
ptCloudScene = pcmerge(pc1, ptCloudAligned, mergeSize);

% Visualize the input images.
figure
subplot(2,2,1)
pcshow(pc1, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('First input image')
drawnow

subplot(2,2,3)
pcshow(pc2, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Second input image')
drawnow

% Visualize the world scene.
subplot(2,2,[2,4])
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow