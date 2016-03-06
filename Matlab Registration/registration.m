clear
clc

% Number of clouds in folder
clouds = 12;

% Load PCD files and put them into XYZ format

pcData = {};
pcColorData = {};
for i = 1:clouds
    string = strcat('360/',num2str(i),'.pcd');
    pcTmp = readPcd(string);
    pc = [pcTmp(:,1),pcTmp(:,2),pcTmp(:,3)];
    pcColor = pcTmp(:,4);
    pcColor = unpackRGBFloat(single(pcColor));
    pcData{i} = pc;
    pcColorData{i} = pcColor;
end

% Convert to point cloud
pc1 = pointCloud(pcData{1},'Color',pcColorData{1});
pc2 = pointCloud(pcData{2},'Color',pcColorData{2});

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

% Store the transformation object that accumulates the transformation.
accumTform = tform;

figure
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

for i = 3:clouds
    ptCloudCurrent = pointCloud(pcData{i},'Color',pcColorData{i});

    % Use previous moving point cloud as reference.
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

    % Apply ICP registration.
    tform = pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);

    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(ptCloudCurrent, accumTform);

    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
    
end

ptCloudFiltered = pcdenoise(ptCloudScene);

% Visualize the world scene.
    hScatter.XData = ptCloudFiltered.Location(:,1);
    hScatter.YData = ptCloudFiltered.Location(:,2);
    hScatter.ZData = ptCloudFiltered.Location(:,3);
    hScatter.CData = ptCloudFiltered.Color;
    drawnow('limitrate')
    
pcshow(ptCloudFiltered, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
        'Parent', hAxes)
title('Updated world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
