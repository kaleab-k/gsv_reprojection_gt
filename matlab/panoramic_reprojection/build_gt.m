% % Simple test. Same location, two images, (1) pitch = 0 and (2) pitch = -20.
% % Test: take a bounding box in (1) and try to reproject it to image (2).
% % Works!
% clc
% clear
% close all

function [GTruth, GTMeta] = build_gt(dir, subdir)

debug_flag = 0;

%%%%%%%%%%%%%%%%% path operations
% add Pano2Context and VOC paths
% addpath('/PanoBasic');
cd('..')
add_path; % Pano2Context functions
cd('panoramic_reprojection')
% addpath([ '~/VOC/VOCdevkit/VOCcode']);

%% Load GTruth and YOLOResult
% dir = '../../dataset/40.4166718,-3.7032952/';
% subdir = 'M:DRIVING_S:608x608';

% [GTruth, GTMeta] = json2gt(dir, subdir);
%% Load the YOLO detections with FOV=60
[GTMeta, GTruthYOLO] = json2struct(dir, [subdir '_GT'], true);

%% Load the GSV Metadata for the FOV=120
[GSVMeta YOLOResult] = json2struct(dir, subdir, true);

% load([ 'yolo.mat'])
% load('yolo_result.mat')

bboxes = struct('x',{},'y',{},'width',{},'height',{},'class',{},'score',{},'cam',{});
pano_bboxes = struct('x',{},'y',{},'width',{},'height',{},'class',{},'score',{},'cam',{});
bboxes_xyz = struct('bbox_xyz', {}, 'score', {}, 'class',{}, 'cam',{});
% bboxes_xyz = {};

%% FIRST TRIAL  
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
    
    if debug_flag
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

    if debug_flag
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

        if ( (im_height - bbox.ymax <=2) || (im_width - bbox.xmax <= 2) || (bbox.xmin <= 2) || (bbox.ymin <= 2) )
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
        if debug_flag
            figure(1),rectangle('Position', [bbox.xmin, bbox.ymin, bbox.xmax-bbox.xmin, bbox.ymax-bbox.ymin],	'EdgeColor','r', 'LineWidth', 3)
        end
        % paint bbox points 
        if debug_flag
            figure(1), plot(bbx_points(:,1),bbx_points(:,2),'y+', 'MarkerSize', 4);
        end

        % project bbox points to panorama image
        [ bbox_xyz, out3DPlane ] = projectPointFromSeparateView(  bbx_points, xyz_GT, gtFOV_rad, im_sz(1), im_sz(2));
        [ bbox_uv ] = xyz2uvN( bbox_xyz);
        [ bbox_coords_pano ] = uv2coords( bbox_uv, sphereW, sphereH);
        
        %% Store the bbox_xyz to the bboxes_xyz list
%         bboxes_xyz{end+1,1} = [bbox_xyz];
%         bboxes_xyz{end,2} = cur_bboxes(i).score;
%         bboxes_xyz{end,3} = cur_bboxes(i).class;
%         bboxes_xyz{end,4} = cur_bboxes(i).cam;
        bboxes_xyz(end+1).bbox_xyz = [bbox_xyz];
        bboxes_xyz(end).score = cur_bboxes(i).score;
        bboxes_xyz(end).class = cur_bboxes(i).class;
        bboxes_xyz(end).cam = cur_bboxes(i).cam;
                      
        if debug_flag
            figure(3), plot(bbox_coords_pano(:,1),bbox_coords_pano(:,2),'r+', 'MarkerSize', 4);
        end
        
    end

        % There is distortion present in the x axis. Probably due to the wide fov.
        % If this is reduced, probably distortion will be lower        
end

GTruthStruct = struct('seqNumber',{}, 'class',{},'x',{}, 'y',{}, 'width',{},'height',{},'confidence',{},'dataSource',{});

%% Reproject bboxes_xyz to FOV=120
for gsvSeq = 2:max([GSVMeta.seqNumber])
    GTruthNMS = struct('seqNumber',{}, 'class',{},'x',{}, 'y',{}, 'width',{},'height',{},'confidence',{},'dataSource',{});
    seq_id = gsvSeq + 1;
    GSVImgSrc =  GSVMeta(seq_id).dataSource;
    GSVImg = double(imread(GSVImgSrc));
    GSVFOV = GSVMeta(seq_id).fov;
    GSVFOV_rad = (GSVFOV/360)*2*pi; % fov in radians
    % heading and pitch in uv format (radians)
    GSV_u_radians = ((GSVMeta(seq_id).heading-180) / 360) * 2 * pi;
    GSV_v_radians = (GSVMeta(seq_id).pitch / 360) * 2 * pi;
    % viewpoints of GSV
    GSV_uv = [GSV_u_radians, GSV_v_radians];
    im_sz = size(GSVImg);
    
    [ xyz_GSV ] = uv2xyzN(GSV_uv);

    % parameters
    im_width = im_sz(2);
    im_height = im_sz(1);

    pano_resolution_factor = round(360/60); %pano_resolution_factor = round(360/90);
    sphereW = im_width * pano_resolution_factor; % keep resolutions
    sphereH = sphereW/2;
    
%     if debug_flag
        h_im_GSV = figure(5);
        imshow(uint8(GSVImg));
        hold on
        title ('NMS on the bboxes')
%         hold off
        h_im_GSV_2 = figure(6);
        imshow(uint8(GSVImg));
        hold on
        title ('GT vs YOLOResult')
%     end
    
    for i = 1:length(bboxes_xyz)
       
        bbox_xyz = bboxes_xyz(i).bbox_xyz;
        
        [bbox_target_ima, valid, division] = projectPoint2SeparateView( bbox_xyz, xyz_GSV, GSVFOV_rad, im_height, im_width);
        
        if(sum(valid)==0)
            continue;
        end
        
        if(~ any(bbox_target_ima(:,1) >= 0 & bbox_target_ima(:,1) <= im_width | bbox_target_ima(:,2) >= 0 & bbox_target_ima(:,2) <= im_height) )
            continue;
        end
        
        phi = 0; 
        if (abs(bbox_target_ima(2,2) - bbox_target_ima(1,2)) > 5)
            if(bbox_target_ima(2,2) > bbox_target_ima(1,2))
                v1 = [bbox_target_ima(2,1),bbox_target_ima(2,2)]-[bbox_target_ima(1,1),bbox_target_ima(1,2)];
                v2 = [bbox_target_ima(2,1),bbox_target_ima(1,2)]-[bbox_target_ima(1,1),bbox_target_ima(1,2)];
                phi = -acos(sum(v1.*v2)/(norm(v1)*norm(v2)));
            else
                v1 = [bbox_target_ima(1,1),bbox_target_ima(1,2)]-[bbox_target_ima(2,1),bbox_target_ima(2,2)];
                v2 = [bbox_target_ima(1,1),bbox_target_ima(2,2)]-[bbox_target_ima(2,1),bbox_target_ima(2,2)];
                phi = acos(sum(v1.*v2)/(norm(v1)*norm(v2)));
            end
            theta =  phi * 180/pi

%         if theta < 89 && theta > -89

            polyin = polyshape(bbox_target_ima(1:4,1)',bbox_target_ima(1:4,2)');

            rot_pnt = [min(bbox_target_ima(1:4,1)) + (max(bbox_target_ima(1:4,1)) - min(bbox_target_ima(1:4,1)))/2, min(bbox_target_ima(1:4,2)) + (max(bbox_target_ima(1:4,2)) - min(bbox_target_ima(1:4,2)))/2];
            poly2 = rotate(polyin,theta,rot_pnt);
            if debug_flag
                figure(7)
                plot([polyin poly2])
                axis equal
            end
%         end
            bbox_target_ima = poly2.Vertices;
        end
        bbox_target_ima(bbox_target_ima < 0) = 0;
        bbox_target_ima(bbox_target_ima(:,1) > im_width,1)=im_width;
        bbox_target_ima(bbox_target_ima(:,2) > im_height,2)=im_height;
        
       
        % extract bounding rectangle on the other image
        xmin_other = min(bbox_target_ima(:,1));
        xmax_other = max(bbox_target_ima(:,1));
        ymin_other = min(bbox_target_ima(:,2));
        ymax_other = max(bbox_target_ima(:,2));
        
         if( (xmax_other - xmin_other < 10) || (ymax_other - ymin_other < 10))
            continue;
         elseif ( (xmax_other - xmin_other == im_width) && (ymax_other - ymin_other == im_height))
                 continue;
         end
         
        
    
        % paint bbox and points
%         if debug_flag
            figure(5), rectangle('Position', [xmin_other, ymin_other, xmax_other-xmin_other,ymax_other-ymin_other],'EdgeColor','r', 'LineWidth', 2);
            figure(5), plot(bbox_target_ima(:,1),bbox_target_ima(:,2),'r+', 'MarkerSize', 4);

%         end
         
        GTruthNMS(end+1).seqNumber = gsvSeq;
        GTruthNMS(end).class = bboxes_xyz(i).class;
        GTruthNMS(end).x = xmin_other;
        GTruthNMS(end).y = ymin_other;
        GTruthNMS(end).width = xmax_other - xmin_other;
        GTruthNMS(end).height = ymax_other - ymin_other;
        GTruthNMS(end).confidence = bboxes_xyz(i).score;
        GTruthNMS(end).dataSource = GSVImgSrc;
       
    end
    
    %% Draw the YOLOResults just for observation
%     if debug_flag
        yolo_bboxes = YOLOResult(ismember([YOLOResult.seqNumber], gsvSeq));
        for j = 1:length(yolo_bboxes)
            bbox =  yolo_bboxes(j);
            figure(6), hold on, rectangle('Position', [bbox.x, bbox.y, bbox.width, bbox.height],'EdgeColor','b', 'LineWidth', 2);
        end
%     end

   
    
    if (~isempty(GTruthNMS))
        detectionResults = yolo2tbl(GTruthNMS, unique({GTruthNMS.class}), max([GTruthNMS.seqNumber]));
        Labels_A = [];
        for i = 1:length(detectionResults.Labels)
            Labels_A = [Labels_A; detectionResults.Labels{i,1}];
        end
        [selectedBboxes, selectedScores, selectedLabels] = selectStrongestBboxMulticlass(cell2mat(detectionResults.Boxes),cell2mat(detectionResults.Scores), Labels_A,  'OverlapThreshold', 0.3);
%         [selectedBboxes, selectedScores] = selectStrongestBbox(cell2mat(detectionResults.Boxes),cell2mat(detectionResults.Scores), 'OverlapThreshold', 0.3);
        for i = 1:size(selectedBboxes,1)
            xmin_nms = selectedBboxes(i,1);
            ymin_nms = selectedBboxes(i,2);
            width_nms =  selectedBboxes(i,3);
            height_nms =  selectedBboxes(i,4);
            score = selectedScores(i,1);
            class = selectedLabels(i,1);
            
            %% Store to the final GTruthStruct
            GTruthStruct(end+1).seqNumber = gsvSeq;
            GTruthStruct(end).class = string(class);
            GTruthStruct(end).x = xmin_nms;
            GTruthStruct(end).y = ymin_nms;
            GTruthStruct(end).width = width_nms;
            GTruthStruct(end).height = height_nms;
            GTruthStruct(end).confidence = score;
            GTruthStruct(end).dataSource = GSVImgSrc;
%             if debug_flag
%             figure(5), plot([xmin_nms ymin_nms; ], [xmin_nms+width_nms ymin_nms+height_nms] ,'r+', 'MarkerSize', 4);

                figure(5), rectangle('Position', [xmin_nms+0.2, ymin_nms+0.2, width_nms, height_nms],'EdgeColor','g', 'LineWidth', 2);
                figure(6), rectangle('Position', [xmin_nms+0.2, ymin_nms+0.2, width_nms, height_nms],'EdgeColor','g', 'LineWidth', 2);
%             end
        end
    end
    
end
GTruth =  struct2gt(GTruthStruct, GTMeta);

end

