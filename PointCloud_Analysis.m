% Filename    : icp_test.m
% Date        : February 18th, 2020
% Programmers : Shreyas Macwan, Gabriel Stewart
% Description : This file imports two point clouds, then performs Point Cloud Registration using ICP Algorithm 

% Add realsense wrapper to path
addpath('C:\Program Files (x86)\Intel RealSense SDK 2.0\matlab\')  
savepath .\pathdef.m

% Reading the point cloud and storing it into a pointcloud object from a .txt file 
pointData = load('.\Training Mold\CadData.txt');
moldCloud  = pointCloud(pointData);

% Collect point cloud from camera
camCloud = pcread('.\Training Mold\CamData.ply');
%rawData = collectData();
%camCloud = pointCloud(rawData);

% Start Timer
tic 

% Denoise the pointclouds before further processing
moldCloud_denoise = pcdenoise(moldCloud);
camCloud_denoise = pcdenoise(camCloud);

% Downsampling both the pointclouds using 'gridAverage' method  
moldCloud_down = pcdownsample(moldCloud_denoise,'gridAverage',8);
camCloud_down= pcdownsample(camCloud_denoise,'gridAverage',0.008);

% Scale the camera cloud data (may be replaced)
A = [1000  0   0       0; ...
     0	1000   0       0; ...
     0	0   1000       0; ...
     0	0   0       1];
 % Apply scale
scale = affine3d(A);
camCloud_scaled = pctransform(camCloud_down,scale);

% Displaying the pointcloud in the 1st Quadrant using 'pcshow' method
subplot(2,2,1);
pcshow(camCloud_scaled);

% Displaying the pointcloud in the 2nd Quadrant using 'pcshow' method
subplot(2,2,2);
pcshow(moldCloud_down);
 
%Displaying the pointcloud in the 3rd Quadrant
%Visualizing both point clouds from above steps, seeing the difference in
%two using pcshowpair
subplot(2,2,3);
pcshowpair(camCloud_scaled,moldCloud_down, 'MarkerSize', 20);

% Use of ICP Pointcloud Registration Algorithm which uses Local Registration scheme  
%tform = pcregistericp(camCloud_down, moldCloud_down,'Metric','pointToPoint');
tform = pcregistercpd(camCloud_scaled, moldCloud_down, 'Transform', 'Rigid', 'Tolerance', 1000);
%tform = pcregisterndt(camCloud_scaled, moldCloud_down, 10);

% Transforming Camera pointcloud with the new transformed Co-ordinates from
% ICP Algorithm
camCloud_tformed = pctransform(camCloud_scaled,tform);

% tform = pcregistericp(camCloud_tformed, moldCloud_down, 'Metric', 'pointToPlane');
% camCloud_tformed_B = pctransform(camCloud_tformed, tform);

%Displaying the pointcloud in the 4th Quadrant
% Visualizing both point clouds from above steps, seeing the difference in two
subplot(2,2,4)
pcshowpair(camCloud_tformed,moldCloud_down, 'MarkerSize', 100)

% End Timer
toc

% %Creates a new instance of the screen
% figure()
% 
% %ptCloudTransformed --- transformed registered cloud 
% %ptCloudTformend_REF -- Reference mold cloud 
% 
% % Getting the XYZ Coordinates point location from both Pointclouds
% xyz_transformed = camCloud_tformed.Location;
% xyz_ref_mold = moldCloud_down.Location;
% 
% % Finding the Euclidean Distance of Reference pointcloud's point to Icp
% % Transformed PointCloud points using pdist2 method
% all_eu_Distance = pdist2(xyz_transformed,xyz_ref_mold);
% 
% % Getting the minimum distance of all the points from the ICP transformed PointCloud  
% min_distance = min(all_eu_Distance,[],2);
% 
% % Initializing an empty array to store points location 
% a=[];
% 
% % Iterating through all the distances and setting and threshold distance 
% for n=1:size(min_distance,1)
%     % If distance is above threshold
%     if min_distance(n)>12.000 
%         %Points Out of the minimum Threshold value, assigning the color RED
%         %to those points
%         %ptCloudTransformed.Color(n,:) = [255 0 0];
% 
%         %Storing point's location in the array generated above 
%         a= cat(1,a,ptCloudTransformed.Location(n,:));
% 
%         % elseif min_distance(n)<22.000 && min_distance(n)>18.000
%         % ptCloudTransformed.Color(n,:) = [0 0 255];
% 
%     else      
%         % Turning Validated Points to color GREEN
%         %ptCloudTransformed.Color(n,:) = [0 255 0];
%    end
% end
% 
% % Creadting pointcloud only for those points which are out of Minimum Threshold 
% newpoint = pointCloud(a);
% 
% % Using 'pcshow' method, Visualizing Transformed pointcloud in the 1st plot of the
% % scrren
% subplot(2,2,1)
% pcshow(ptCloudTransformed);
% 
% % Using 'pcshow' method, Visualizing the differentiated pointcloud in the 2nd plot of the
% % scrren
% subplot(2,2,2)
% pcshow(newpoint)




