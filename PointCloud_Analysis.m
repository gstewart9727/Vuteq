% Filename    : icp_test.m
% Date        : February 18th, 2020
% Programmers : Shreyas Macwan, Gabriel Stewart
% Description : This file imports two point clouds, then performs Point Cloud Registration using ICP Algorithm 

% Reading the point cloud and storing it into a pointcloud object from a
% .PLY file 

refMold = pcread('C:\Users\Smacwan1933\Downloads\mesh2.ply');       % --- Reading Reference PointCloud PLY file 
refCamera = pcread('C:\Users\Smacwan1933\Downloads\cleanface.ply'); % --- Reading Camera PointCloud PLY file

% Downsampling both the pointclouds using 'gridAverage' method  
refMold_down = pcdownsample(refMold,'gridAverage',15);
cloudout3= pcdownsample(refCamera,'gridAverage',0.010);

% Denoising a pointcloud generated from a RealSense Camera
refCam_denoise = pcdenoise(cloudout3,'NumNeighbors',8,...
    'Threshold',0.1);

% For a quality registration, scaling the pointcloud (RealSense Camera)
% Create matrix for scaling
A = [2100  0   0       0; ...
     0	2100   0       0; ...
     0	0   2100       0; ...
     0	0   0       1];
 % Apply scale
tform1 = affine3d(A);

% Scaling and Transformation of Camera pointcloud using the 'pctransform' method 
ptCloudTformed = pctransform(refCam_denoise,tform1);

% Displaying the pointcloud in the 1st Quadrant using 'pcshow' method
subplot(2,2,1)
pcshow(ptCloudTformed)

% Rotating reference pointcloud to 90 degrees 
% Evaluate constant
n= pi/2;
% Create matrix for transformation
A2 =[1  0   0       0; ...
     0	cos(n)    sin(n)       0; ...
     0	-sin(n)   cos(n)       0; ...
     0	0   0       1];
% Apply transformation
tform2 = affine3d(A2);
 
% Scaling and Transformation of Reference pointcloud using the 'pctransform' method 
ptCloudTformend_REF = pctransform(refMold_down,tform2);

% Displaying the pointcloud in the 2nd Quadrant using 'pcshow' method
subplot(2,2,2)
pcshow(ptCloudTformend_REF)
 
%Displaying the pointcloud in the 3rd Quadrant
%Visualizing both point clouds from above steps, seeing the difference in
%two using pcshowpair
subplot(2,2,3)
pcshowpair(ptCloudTformed,ptCloudTformend_REF, 'MarkerSize', 20);

% tic  / toc for see the time required for operation
% Use of ICP Pointcloud Registration Algorithm which uses Local Registration scheme  
tic  
reg_cloud= pcregistericp(ptCloudTformed,ptCloudTformend_REF,'Metric','pointToPoint');
toc

% Transforming Camera pointcloud with the new transformed Co-ordinates from
% ICP Algorithm
ptCloudTransformed = pctransform(ptCloudTformed,reg_cloud);

%Displaying the pointcloud in the 4th Quadrant
% Visualizing both point clouds from above steps, seeing the difference in two
subplot(2,2,4)
pcshowpair(ptCloudTransformed,ptCloudTformend_REF, 'MarkerSize', 100)

%Creates a new instance of the screen
figure()

%ptCloudTransformed --- transformed registered cloud 
%ptCloudTformend_REF -- Reference mold cloud 

% Getting the XYZ Coordinates point location from both Pointclouds
xyz_transformed = ptCloudTransformed.Location;
xyz_ref_mold = ptCloudTformend_REF.Location;

% Finding the Euclidean Distance of Reference pointcloud's point to Icp
% Transformed PointCloud points using pdist2 method
all_eu_Distance = pdist2(xyz_transformed,xyz_ref_mold);

% Getting the minimum distance of all the points from the ICP transformed PointCloud  
min_distance = min(all_eu_Distance,[],2);

% Initializing an empty array to store points location 
a=[];

% Iterating through all the distances and setting and threshold distance 
for n=1:size(min_distance,1)
    % If distance is above threshold
    if min_distance(n)>12.000 
        %Points Out of the minimum Threshold value, assigning the color RED
        %to those points
        ptCloudTransformed.Color(n,:) = [255 0 0];

        %Storing point's location in the array generated above 
        a= cat(1,a,ptCloudTransformed.Location(n,:));

        % elseif min_distance(n)<22.000 && min_distance(n)>18.000
        % ptCloudTransformed.Color(n,:) = [0 0 255];

    else      
        % Turning Validated Points to color GREEN
        ptCloudTransformed.Color(n,:) = [0 255 0];
   end
end

% Creadting pointcloud only for those points which are out of Minimum Threshold 
newpoint = pointCloud(a);

% Using 'pcshow' method, Visualizing Transformed pointcloud in the 1st plot of the
% scrren
subplot(2,2,1)
pcshow(ptCloudTransformed);

% Using 'pcshow' method, Visualizing the differentiated pointcloud in the 2nd plot of the
% scrren
subplot(2,2,2)
pcshow(newpoint)




