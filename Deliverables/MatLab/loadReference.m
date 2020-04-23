% Function loads Reference point cloud from CAD file and performs downsamping.

% Function returns downsampled point cloud, transformed point cloud and the transformed matrix.

function [ref_down_sampled_ref,ptCloudTformed_REF,tform2] = loadReference()
                    
                    % Loads .ply file as point cloud object.
                    % PLY files are used to store the 3D - XYZ co-ordinates data. 
                    ptcloud_REF  = pcread('Main-Refined.ply');
                    
                    % 60 degree rotation parameter 
                    n = pi/3;
                    
                    % Rotation along z axis based on degree specified above. 
                    A1 = [ cos(n)  -sin(n)      0      0; ...
                            sin(n)   cos(n)     0      0; ...
                             0        0         1      0; ... 
                             0	      0         0      1];
                             
                    % 5 degree rotation parameter         
                    n2 = 5;
                    
                    % Rotation along x axis based on degree specified above.
                    A2 = [ 1   0          0           0; ...
                           0   cosd(n2)   -sind(n2)   0; ...
                           0   sind(n2)   cosd(n2)    0; ... 
                           0   0          0           1];
                    % tform= affine3d(AX);
                    
                    % Generates rotation matrix object applying which define the rotation parameters along z and x axis.
                    tform2 = affine3d(A1*A2);

                    % Downsampling the Reference point cloud based on gridAverage method.
                    ref_down_sampled_ref = pcdownsample(ptcloud_REF,'gridAverage',3.5); % default is 3.5....keep the range from 2.5 - 3.5
                    
                    % Applying transformations(Rotation) on the downsampled point cloud.
                    ptCloudTformed_REF = pctransform(ref_down_sampled_ref,tform2);



end
