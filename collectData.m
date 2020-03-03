function pcData = collectData()

    pcl_obj = realsense.pointcloud();
    
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();

    % Start streaming on an arbitrary camera with default settings
    pipe.start();

    % Align to color data
    align_to = realsense.stream.color;
    alignedFs = realsense.align(align_to);
    
    % Get frames. We discard the first couple to allow
    % the camera time to settle
    for i = 1:50
        fs = pipe.wait_for_frames();
    end
    
    % Stop streaming
    pipe.stop();

    % Select depth frame
    aligned_frames = alignedFs.process(fs);
    depth = aligned_frames.get_depth_frame();

    % Gather points from depth data
    points = pcl_obj.calculate(depth);
    
    % Gather vertices from point data
    pcData = points.get_vertices();
    pcshow(pcData)