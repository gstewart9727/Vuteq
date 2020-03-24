function [ref_down_sampled_ref,ptCloudTformed_REF,tform2] = loadReference()

                    ptcloud_REF  = pcread('Main-Refined.ply');
                    n = pi/3;
                    A1 = [ cos(n)  -sin(n)      0      0; ...
                            sin(n)   cos(n)     0      0; ...
                             0        0         1      0; ... 
                             0	      0         0      1];
                    n2 = 5;
                    A2 = [ 1   0          0           0; ...
                           0   cosd(n2)   -sind(n2)   0; ...
                           0   sind(n2)   cosd(n2)    0; ... 
                           0   0          0           1];
                    % tform= affine3d(AX);

                    tform2 = affine3d(A1*A2);


                    ref_down_sampled_ref = pcdownsample(ptcloud_REF,'gridAverage',3.5); % default is 3.5....keep the range from 2.5 - 3.5
                    ptCloudTformed_REF = pctransform(ref_down_sampled_ref,tform2);



end
