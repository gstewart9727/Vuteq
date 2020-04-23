% Filename    : FULLY_AUTO.m
% Programmer  : Shreyas Macwan
% Description : This script connects to c++ libraries to connect
%               with Intel Realsense d435 camera module, capures depth frames, does
%               certain cropping, converts it into pointcloud, performs point cloud registration
%               with Reference pointcloud and find deviation between them.


% Starts timer
tic
% Checking for camera initially
if ~exist("isConnected", "var") || isConnected  == 0
    
           % pipeine object to connect and start Intel RealSense camera 
           pipe = realsense.pipeline();
           profile =  pipe.start();
           dev =  profile.get_device();

           context = realsense.context();
           devicehub = realsense.device_hub(context);
           isConnected = devicehub.is_connected(dev);

           %profile,isConnected] = checkConnection();
           
 else
           %No action needed 
 end
 

%define point cloud object

pcl_obj = realsense.pointcloud();

%colorizer = realsense.colorizer();
%Initializing decimation filter 
deci_filter = realsense.decimation_filter(2.00);

%Initializing spatial filter 
%spatial_filter =  realsense.spatial_filter(1.00,22.00,4,2);
spatial_filter =  realsense.spatial_filter(1.00,50.00,5,2);

%Start streaming on an arbvvvitrary camera with default settings

%end

%profiles = pipe.start();

%Starting camera's color frame
align_to = realsense.stream.color;

% Preparing color frame and depth frame 
alignedFs = realsense.align(align_to);
%Get frames. Discard the first several to allow

%the camera time to settle
% for i = 1:5

% Fetching a live frame
frames = pipe.wait_for_frames();


% end
% Processsing received frame with color aligned frame 
aligned_frames = alignedFs.process(frames);

%Gives depth frame out of aligned frame 
depth = aligned_frames.get_depth_frame();   

%depth_without = depth;
%Applying decimaation and spatial filters on received depth frame
depth = deci_filter.process(depth);
depth = spatial_filter.process(depth);

%color = frames.get_color_frame();
%pcl_obj.map_to(color);


points = pcl_obj.calculate(depth);
%points_without = pcl_obj.calculate(depth_without);

%Fetching X-Y-Z Co-ordinated of the 3D scene 
vertices = points.get_vertices();
%vetices_without = points_without.get_vertices();

% Gathering Reference point cloud data
if exist('ptCloudTformed_REF','var')
    
    
    ptCloudTformed_REF = pctransform(ref_down_sampled,tform2);

else
    % loading Reference cloud object if initially not exist
    [ref_down_sampled,ptCloudTformed_REF,tform2]  = loadReference();
end


% Applying some transformations on camera point cloud 

ptCloudTformed = cameraDownsample_Transform(vertices);

% Registering the point cloud ....
% First CPD and later ICP Registration Algorithm .....

if exist('cpd_reg','var')

    Final_cam = pctransform(ptCloudTformed,cpd_reg);

else
    
    % CPD Registration 
    cpd_reg = pcregistercpd(ptCloudTformed,ptCloudTformed_REF,'Transform','Rigid','MaxIterations',15);  % default is 25

    Final_cam = pctransform(ptCloudTformed,cpd_reg);
end


% ICP Registration
reg_cloud= pcregistericp(Final_cam,ptCloudTformed_REF,'Metric','pointToPoint','MaxIterations',10);  % default is 10

 

%Applying Transformation based on transforming values provided by ICP 
ptCloudTransformed_FINAL = pctransform(Final_cam,reg_cloud);

% figure
% pcshowpair(ptCloudTransformed_FINAL,ptCloudTformed_REF,'MarkerSize',50);


% Deviation Analysis 

ptCloudTransformed_FINAL = findDeviation(ptCloudTransformed_FINAL,ptCloudTformed_REF);

%Reads elapsed time
toc

% Creates figure window to see the result in it 
figure

% Creates a visualization depicting the differences between two
% pointclouds(Reference and Camera generated)
pcshowpair(ptCloudTransformed_FINAL,ptCloudTformed_REF)



% This function will be used to show errors(deviation detected) with Alert ....
% Later on will try to introduce some slert threshold in order to invoke
% this funtion ....can introduce and array and store result in that to
% provide threshold values 

% size(myDeviatedPoints,1)

% function dimention =  displaymyAlert()
% 
%     f = uifigure('Position',[680 558 400 180]);
%     dimention = f.Position;
%     uialert(f,'Anamoly Detected','Deviation Analysis');
% 
% end


