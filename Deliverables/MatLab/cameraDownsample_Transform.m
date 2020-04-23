% Function to downsample camera generated point cloud.
function ptCloudTformed =  cameraDownsample_Transform(vertices)
            
            % n defines degree parameter for Rotating the cloud 
            n = 170.0;
            
            % y-axis rotation matrix  
            A =[ cosd(n)  0 sind(n)   0; ...
                    0   1    0    0; ...
                     -sind(n)  0   cosd(n)   0; ... 
                     0	        0       0       1];
            % tform1 contains transormation matrix after rotation
            tform1= affine3d(A);
            
            %Converting matrix vertices to point Cloud Object 
            CAM = pctransform(pointCloud(vertices),tform1);
            n2 = 30.0;
            A2 =[ 1   0           0          0; ...
                  0   cosd(n2)    -sind(n2)  0; ...
                  0   sind(n2)    cosd(n2)   0; ... 
                  0	  0           0          1];
            tform1= affine3d(A2);
            CAM2 = pctransform(CAM,tform1);
            
            % Extacting XYZ co-ordinates from point cloud object 
            points = CAM2.Location;
            
            % Static mechanism to crop ROI(Mold surface to be observed).
            points = points(points(:,3)>-0.2,:);
            points = points(points(:,3)<-0.16,:);
            points = points(points(:,1)>-0.15,:);
            points = points(points(:,1)<0.05,:);
            points = points(points(:,3)>-0.19,:);
            
            % Point cloud object creation from ROI points 
            ptCloudTformed_original =pointCloud(points);
            %ptCloudTformed = pcdownsample(ptCloudTformed_original,'gridAverage',0.0020);
            
            % Performs downsampling on the point cloud 
            %% Downsampling reduces the number of redundant points from the point cloud which enhances the registration process. 
            ptCloudTformed = pcdownsample(ptCloudTformed_original,'gridAverage',0.0038);  % default is 0.0035
            
            % Scaling the live camera point cloud to match with the CAD reference point cloud for acurate registration. 
            A3 = [1000   0     0       0; ...
                   0	1000   0       0; ...
                   0	0     1000     0; ...
                   0	0      0       1];
            % tform1 contains transormation matrix after scaling       
            tform1 = affine3d(A3);
            
            % Final point cloud after applying scaling on the point cloud.
            ptCloudTformed = pctransform(ptCloudTformed,tform1);
end
