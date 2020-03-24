function ptCloudTformed =  cameraDownsample_Transform(vertices)

            n = 170.0;
            A =[ cosd(n)  0 sind(n)   0; ...
                    0   1    0    0; ...
                     -sind(n)  0   cosd(n)   0; ... 
                     0	        0       0       1];
                 tform1= affine3d(A);
            CAM = pctransform(pointCloud(vertices),tform1);
            n2 = 30.0;
            A2 =[ 1   0           0          0; ...
                  0   cosd(n2)    -sind(n2)  0; ...
                  0   sind(n2)    cosd(n2)   0; ... 
                  0	  0           0          1];
            tform1= affine3d(A2);
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
            A3 = [1000   0     0       0; ...
                   0	1000   0       0; ...
                   0	0     1000     0; ...
                   0	0      0       1];
            tform1 = affine3d(A3);
            ptCloudTformed = pctransform(ptCloudTformed,tform1);
end