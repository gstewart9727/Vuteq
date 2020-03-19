
%pause on 
if ~exist("isConnected", "var") || isConnected  == 0
    
           
           pipe = realsense.pipeline();
           profile =  pipe.start();
           dev =  profile.get_device();

           context = realsense.context();
           devicehub = realsense.device_hub(context);
           isConnected = devicehub.is_connected(dev);
           
 else
     
 end
 

%define point cloud object

pcl_obj = realsense.pointcloud();

%define colorizer to give point cloud color

%colorizer = realsense.colorizer();
deci_filter = realsense.decimation_filter(2.00);
%spatial_filter =  realsense.spatial_filter(1.00,22.00,4,2);
spatial_filter =  realsense.spatial_filter(1.00,50.00,5,2);

%Start streaming on an arbvvvitrary camera with default settings

%end

%profiles = pipe.start();

align_to = realsense.stream.color;


alignedFs = realsense.align(align_to);
%Get frames. Discard the first several to allow

%the camera time to settle
% for i = 1:5

    frames = pipe.wait_for_frames();

% end


 

aligned_frames = alignedFs.process(frames);

depth = aligned_frames.get_depth_frame();   

%depth_without = depth;
depth = deci_filter.process(depth);

depth = spatial_filter.process(depth);



%color = frames.get_color_frame();



%pcl_obj.map_to(color);



points = pcl_obj.calculate(depth);
%points_without = pcl_obj.calculate(depth_without);

vertices = points.get_vertices();
%vetices_without = points_without.get_vertices();


pcl_test = pointCloud(vertices);

new_vertices =  vertices;
%tex_coords = points.get_texture_coordinates();

%colordata=color.get_data();

%img = permute(reshape(colordata',[3,color.get_width(),color.get_height()]),[3 2 1]);

%imshow(img);



% Gathering Reference point cloud data
if exist('ptcloud_REF','var')
    ptCloudTformend_REF = pctransform(ref_down_sampled,tform2);

else
    ptcloud_REF  = pcread('Main-Refined.ply');
    n=pi/3;
    A2 =[ cos(n)  -sin(n)  0    0; ...
    sin(n)   cos(n)     0    0; ...
     0         0         1   0; ... 
     0	        0       0       1];
 
     n2 = 5;
    AX =[ 1  0         0           0; ...
       0  cosd(n2)   -sind(n2)    0; ...
      0  sind(n2)    cosd(n2)    0; ... 
      0	   0         0         1];
    % tform= affine3d(AX);
 
    tform2 = affine3d(A2*AX);
    
    
    ref_down_sampled = pcdownsample(ptcloud_REF,'gridAverage',3.5); % default is 2.5
    ptCloudTformend_REF = pctransform(ref_down_sampled,tform2);
end





% Applying some transformation on camera point cloud 

n = 170.0;
A2 =[ cosd(n)  0 sind(n)   0; ...
        0   1    0    0; ...
         -sind(n)  0   cosd(n)   0; ... 
         0	        0       0       1];
     tform1= affine3d(A2);
CAM = pctransform(pointCloud(vertices),tform1);
n = 30.0;
A3 =[ 1 0 0   0; ...
        0 cosd(n)   -sind(n)    0; ...
         0 sind(n)  cosd(n)   0; ... 
         0	        0       0       1];
tform1= affine3d(A3);
CAM2 = pctransform(CAM,tform1);
points = CAM2.Location;
points = points(points(:,3)>-0.2,:);
points = points(points(:,3)<-0.16,:);
points = points(points(:,1)>-0.15,:);

points = points(points(:,1)<0.05,:);


% testing this below line 
points = points(points(:,3)>-0.19,:);



ptCloudTformed_original =pointCloud(points);

%ptCloudTformed = pcdownsample(ptCloudTformed_original,'gridAverage',0.0020);

ptCloudTformed = pcdownsample(ptCloudTformed_original,'gridAverage',0.0038);  % default is 0.0035

A = [1000     0   0       0; ...
     0	1000   0       0; ...
     0	0   1000       0; ...
     0	0   0       1];

tform1 = affine3d(A);
ptCloudTformed = pctransform(ptCloudTformed,tform1);



% Registering the point cloud ....
% First CPD and later ICP.....

if exist('cpd_reg','var')
    
    Final_cam = pctransform(ptCloudTformed,cpd_reg);
    
else
    
    cpd_reg = pcregistercpd(ptCloudTformed,ptCloudTformend_REF,'Transform','Rigid','MaxIterations',15);  % default is 25
    
    Final_cam = pctransform(ptCloudTformed,cpd_reg);
end
 
 


reg_cloud= pcregistericp(Final_cam,ptCloudTformend_REF,'Metric','pointToPoint','MaxIterations',5);  % default is 10




ptCloudTransformed_FINAL = pctransform(Final_cam,reg_cloud);

figure
pcshowpair(ptCloudTransformed_FINAL,ptCloudTformend_REF,'MarkerSize',50);


% Deviation Analysis

tic 

XYZ_CAM = ptCloudTransformed_FINAL.Location;
XYZ_REF = ptCloudTformend_REF.Location;

all_eu_Distance = pdist2(XYZ_CAM,XYZ_REF);

min_distance = min(all_eu_Distance,[],2);

numOfPoints = ptCloudTransformed_FINAL.Count;
numOfPointsInMatrix = zeros(numOfPoints,3);
numOfPointsInMatrix =uint8(numOfPointsInMatrix);
ptCloudTransformed_FINAL.Color = numOfPointsInMatrix;

myDeviatedPoints = [];

for n = 1:size(min_distance,1)
    if(min_distance(n)>6.9)  % keep default threshold between  5.9 - 6.9
        ptCloudTransformed_FINAL.Color(n,:) = [255,0,0];
         myDeviatedPoints = cat(1,myDeviatedPoints,ptCloudTransformed_FINAL.Location(n,:));
         if myDeviatedPoints > 100
             
         end
    
    else
        ptCloudTransformed_FINAL.Color(n,:) = [0,255,0];
    end
end



figure
pcshowpair(ptCloudTransformed_FINAL,ptCloudTformend_REF)
toc


% This function will be used to show errors(deviation detected) ....
% Later on will try to introduce some slert threshold in order to invoke
% this funtion ....can introduce and array and store result in that to
% provide threshold values 

% size(myDeviatedPoints,1)

function dimention =  displaymyAlert()
44444
    f = uifigure('Position',[680 558 400 180]);
    dimention = f.Position;
    uialert(f,'Anamoly Detected','Deviation Analysis');

end


