%==========================================================================
function [bboxes_xyz] = getBBoxesFromGT(dir, subdir)

    debug_flag = 0;
    bboxes_xyz = struct('bbox_xyz', {}, 'score', {}, 'class',{}, 'cam',{});
    bboxes = struct('x',{},'y',{},'width',{},'height',{},'class',{},'score',{},'cam',{});
    pano_bboxes = struct('x',{},'y',{},'width',{},'height',{},'class',{},'score',{},'cam',{});

    %% Load the YOLO detections with FOV=60
    [GTMeta, GTruthYOLO] = json2struct(dir, [subdir '_GT'], true);

    for gtSeq = 0:max([GTruthYOLO(:).seqNumber])
        seq_id = gtSeq + 1;
        gtImgSrc =  GTMeta(seq_id).dataSource;
        gtImg = double(imread(gtImgSrc));
        gtFOV = GTMeta(seq_id).fov;
        gtFOV_rad = (gtFOV/360)*2*pi; % fov in radians
        % heading and pitch in uv format (radians)
        gt_u_radians = ((GTMeta(seq_id).heading-180) / 360) * 2 * pi;
        gt_v_radians = (GTMeta(seq_id).pitch / 360) * 2 * pi;
        % viewpoints of GT
        gt_uv = [gt_u_radians, gt_v_radians];
        im_sz = size(gtImg);

        % parameters
        im_width = im_sz(2);
        im_height = im_sz(1);

        pano_resolution_factor = round(360/gtFOV); %pano_resolution_factor = round(360/90);
        sphereW = im_width * pano_resolution_factor; % keep resolutions
        sphereH = sphereW/2;

        if debug_flag == 1
            h_im_GT = figure(1);
            imshow(uint8(gtImg));
            hold on
            title ('original image with sample bbox')
        end

        curr_seq_det = [GTruthYOLO(ismember([GTruthYOLO.seqNumber],gtSeq))];
        cur_bboxes = [];

        % Skip if no bboxes in the scene
        if (size(curr_seq_det,1) == 0) 
            continue;
        end

        for i = 1:size(curr_seq_det,1)
            bbox = curr_seq_det(i);
            cur_bboxes(end+1,1).x = bbox.x;
            cur_bboxes(end,1).y = bbox.y;
            cur_bboxes(end,1).width = bbox.width;
            cur_bboxes(end,1).height = bbox.height;
            cur_bboxes(end,1).class = bbox.class;
            cur_bboxes(end,1).score = bbox.confidence;
            cur_bboxes(end,1).cam = gtSeq;
            bboxes(end+1,1) = cur_bboxes(end,1);
        end

        % convert center of images in xyz format. check what this means exactly
        [ xyz_GT ] = uv2xyzN(gt_uv);

        % projections of the images to panorama to see if they fit
        [sphereImg_GT, validMap_GT] = im2Sphere(gtImg, gtFOV_rad, sphereW, sphereH, gt_u_radians, gt_v_radians );

        if debug_flag == 1
            h_pano = figure(3);
            imshow(uint8(sphereImg_GT));
            hold on
            figure
            imagesc(validMap_GT);
        end
        % try with sample bounding boxes
        % bbox_array = gTruth.LabelData.person{1,1};
        for i = 1:size(cur_bboxes,1) % loop over the bboxes

            % bbox in another format
            bbox.xmin = cur_bboxes(i).x;
            bbox.ymin = cur_bboxes(i).y;
            bbox.xmax = bbox.xmin + cur_bboxes(i).width;
            bbox.ymax = bbox.ymin + cur_bboxes(i).height;

            if ( (im_height - bbox.ymax <=10) || (im_width - bbox.xmax <= 10) || (bbox.xmin <= 10) || (bbox.ymin <= 10) )
                continue;
            end

            bbx_points = [ (bbox.xmin) (bbox.xmax) (bbox.xmax) (bbox.xmin);
                           (bbox.ymin) (bbox.ymin) (bbox.ymax) (bbox.ymax)]; 
            bbx_points = bbx_points';

            % extract also upper and lower medium points (to avoid problems with big distorted objects)
            upper_middle_x = round((bbox.xmin + bbox.xmax)/2);
            upper_middle_y = bbox.ymin;
            lower_middle_x = round((bbox.xmin + bbox.xmax)/2);
            lower_middle_y = bbox.ymax;
            bbx_points(5,:) = [upper_middle_x upper_middle_y];
            bbx_points(6,:) = [lower_middle_x lower_middle_y];


            % paint bbox 
            if debug_flag == 1
                figure(1),rectangle('Position', [bbox.xmin, bbox.ymin, bbox.xmax-bbox.xmin, bbox.ymax-bbox.ymin],	'EdgeColor','r', 'LineWidth', 3)
            end
            % paint bbox points 
            if debug_flag == 1
                figure(1), plot(bbx_points(:,1),bbx_points(:,2),'y+', 'MarkerSize', 4);
            end

            % project bbox points to panorama image
            [ bbox_xyz, ~ ] = projectPointFromSeparateView(  bbx_points, xyz_GT, gtFOV_rad, im_sz(1), im_sz(2));
            [ bbox_uv ] = xyz2uvN( bbox_xyz);
            [ bbox_coords_pano ] = uv2coords( bbox_uv, sphereW, sphereH);

            %% Store the bbox_xyz to the bboxes_xyz list
            bboxes_xyz(end+1).bbox_xyz = [bbox_xyz];
            bboxes_xyz(end).score = cur_bboxes(i).score;
            bboxes_xyz(end).class = cur_bboxes(i).class;
            bboxes_xyz(end).cam = cur_bboxes(i).cam;

            if debug_flag == 1
                figure(3), plot(bbox_coords_pano(:,1),bbox_coords_pano(:,2),'r+', 'MarkerSize', 4);
            end

        end
    end
end
