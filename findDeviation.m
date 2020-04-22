% Function to find the deviation from the registered point cloud based on Threshold value
% Function takes two parameters 1. Camera point cloud, 2. Reference point Cloud (CAD file)
                                
function ptCloudTransformed_FINAL = findDeviation(ptCloudTransformed_FINAL,ptCloudTformed_REF)
                                    
                                    % Extract XYZ points from Camera pointcloud(Transformed cloud)
                                    XYZ_CAM = ptCloudTransformed_FINAL.Location;
                                    % Extract XYZ points from Reference pointcloud
                                    XYZ_REF = ptCloudTformed_REF.Location;
                                    
                                    % pdist2 function finds all the distances between the points from 
                                    % transformed camera pointcloud and reference point cloud 
                                    all_eu_Distance = pdist2(XYZ_CAM,XYZ_REF);
                                    
                                    % Calculating the minimum distance from all the distances
                                    min_distance = min(all_eu_Distance,[],2);
                                    
                                    % Counting how many points transformed camera point cloud contains 
                                    numOfPoints = ptCloudTransformed_FINAL.Count;
                                    
                                    % Creating empty matrix based on the number of points in transformed camera point cloud 
                                    numOfPointsInMatrix = zeros(numOfPoints,3);
                                    
                                    % Double to integer conversion 
                                    numOfPointsInMatrix = uint8(numOfPointsInMatrix);
                                    ptCloudTransformed_FINAL.Color = numOfPointsInMatrix;

                                    myDeviatedPoints = [];
                                    
                                    % This loop iterates though every point from the minimum distance matrix
                                    for n = 1:size(min_distance,1)
                                        
                                        % Specifying Threshold value. Points above this threshold will be denoted as Anomaly 
                                        if(min_distance(n)>6.5)  % keep default threshold between  5.9 - 6.9
                                            
                                            % Points above the threhold are converted into red in color to easy visualization
                                            ptCloudTransformed_FINAL.Color(n,:) = [255,0,0];
                                             myDeviatedPoints = cat(1,myDeviatedPoints,ptCloudTransformed_FINAL.Location(n,:));
                                             if myDeviatedPoints > 100

                                             end

                                        else
                                            ptCloudTransformed_FINAL.Color(n,:) = [0,255,0];
                                        end
                                    end

end
