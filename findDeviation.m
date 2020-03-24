function ptCloudTransformed_FINAL = findDeviation(ptCloudTransformed_FINAL,ptCloudTformed_REF)

                                    XYZ_CAM = ptCloudTransformed_FINAL.Location;
                                    XYZ_REF = ptCloudTformed_REF.Location;

                                    all_eu_Distance = pdist2(XYZ_CAM,XYZ_REF);
                                    min_distance = min(all_eu_Distance,[],2);
                                    numOfPoints = ptCloudTransformed_FINAL.Count;
                                    numOfPointsInMatrix = zeros(numOfPoints,3);
                                    numOfPointsInMatrix = uint8(numOfPointsInMatrix);
                                    ptCloudTransformed_FINAL.Color = numOfPointsInMatrix;

                                    myDeviatedPoints = [];

                                    for n = 1:size(min_distance,1)
                                        if(min_distance(n)>6.5)  % keep default threshold between  5.9 - 6.9
                                            ptCloudTransformed_FINAL.Color(n,:) = [255,0,0];
                                             myDeviatedPoints = cat(1,myDeviatedPoints,ptCloudTransformed_FINAL.Location(n,:));
                                             if myDeviatedPoints > 100

                                             end

                                        else
                                            ptCloudTransformed_FINAL.Color(n,:) = [0,255,0];
                                        end
                                    end

end
