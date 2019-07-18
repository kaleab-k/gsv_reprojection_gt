% Simple test. Same location, two images, (1) pitch = 0 and (2) pitch = -20.
% Test: take a bounding box in (1) and try to reproject it to image (2).
% Works!
clc
clear
close all
debug_flag = 0;
fixProp = 'pitch';

%%%%%%%%%%%%%%%%% path operations
% add Pano2Context and VOC paths
% addpath('/PanoBasic');
cd('..')
add_path; % Pano2Context functions
cd('panoramic_reprojection')
% addpath([ '~/VOC/VOCdevkit/VOCcode']);

%% Load GTruth and YOLOResult
load([ 'yolo.mat'])
load('yolo_result.mat')


% test = 'cars';
% imHoriFOV_degrees = 60;
bboxes = struct('x',{},'y',{},'width',{},'height',{},'class',{},'score',{},'cam',{});
pano_bboxes = struct('x',{},'y',{},'width',{},'height',{},'class',{},'score',{},'cam',{});

%% FIRST TRIAL  
for gtSeq = 0:max([YOLOResult(:).seqNumber])
    seq_id = gtSeq + 1;
    gtImgSrc =  GSVMeta(seq_id).dataSource;
    gtImg = double(imread(gtImgSrc));
    gtFOV = GSVMeta(seq_id).fov;
    gtFOV_rad = (gtFOV/360)*2*pi; % fov in radians
    % heading and pitch in uv format (radians)
    gt_u_radians = ((GSVMeta(seq_id).heading-180) / 360) * 2 * pi;
    gt_v_radians = (GSVMeta(seq_id).pitch / 360) * 2 * pi;
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
    
    curr_seq_det = [YOLOResult(ismember([YOLOResult.seqNumber],gtSeq))];
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
    
%     cell2struct(cellfun(@vertcat,struct2cell(cur_bboxes),struct2cell(bboxes),'uni',0),fieldnames(cur_bboxes),1);

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

        bbx_points = [ (bbox.xmin) (bbox.xmax) (bbox.xmin) (bbox.xmax);
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
%         disp(bbox_coords_pano);
%         bbox_nms = [bbox_coords_pano(1,1) bbox_coords_pano(1,2)...
%                     bbox_coords_pano(2,1) bbox_coords_pano(2,2)... 
%                     bbox_coords_pano(3,1) bbox_coords_pano(3,2)...
%                     bbox_coords_pano(4,1) bbox_coords_pano(4,2)...
%                     cur_bboxes(i).class  cur_bboxes(i).score cur_bboxes(i).cam];
          bbox_nms.x = min(bbox_coords_pano(:,1));
          bbox_nms.y = min(bbox_coords_pano(:,2));
          bbox_nms.width = max(bbox_coords_pano(:,1)) - bbox_nms.x;
          bbox_nms.height = max(bbox_coords_pano(:,2)) - bbox_nms.y;
          bbox_nms.class = cur_bboxes(i).class;
          bbox_nms.score = cur_bboxes(i).score;
          bbox_nms.cam = cur_bboxes(i).cam;
        pano_bboxes(end+1,1) = bbox_nms;
                
        if debug_flag
            figure(3), plot(bbox_coords_pano(:,1),bbox_coords_pano(:,2),'r+', 'MarkerSize', 4);
        end
        % store bbox
%             gtr_idx = testIdx + (i-1);
        
    end

        % There is distortion present in the x axis. Probably due to the wide fov.
        % If this is reduced, probably distortion will be lower        
end
[pano,nms_bboxes] = gsv_pano_nms(pano_bboxes);
figure(4), imshow(uint8(pano));
for i = 1:length(pano_bboxes)
    box = pano_bboxes(i);
    hold on, rectangle('Position', [box.x, box.y, box.width, box.height],'EdgeColor','g', 'LineWidth', 3)
end

for i = 1:length(nms_bboxes)
    box = nms_bboxes(i);
    hold on, rectangle('Position', [box.x+1, box.y+1, box.width,box.height],'EdgeColor','r', 'LineWidth', 3)
end
% for i = 1:length(nms_bboxes)
%     box(:,1) = [nms_bboxes(:).x; [[nms_bboxes(:).x]+[nms_bboxes(:).width]]' ];
%     box(:,2) = [nms_bboxes(:).y; [[nms_bboxes(:).y]+[nms_bboxes(:).height]]' ];
%     box(:,1) = reshape(box, 2,4)';
%     figure(4), plot(box(:,1), box(:,2), 'g+', 'MarkerSize',2);
%     rectangle('Position', [box(1), box(2), box(7)-box(1),box(8)-box(2)],'EdgeColor','g', 'LineWidth', 3)
% end



%% Reproject pano boxes to perspective boxes 
for gtSeq = 0:max([YOLOResult(:).seqNumber])
    seq_id = gtSeq + 1;
    gtImgSrc =  GSVMeta(seq_id).dataSource;
    gtImg = double(imread(gtImgSrc));
    gtFOV = GSVMeta(seq_id).fov;
    gtFOV_rad = (gtFOV/360)*2*pi; % fov in radians
    % heading and pitch in uv format (radians)
    gt_u_radians = ((GSVMeta(seq_id).heading-180) / 360) * 2 * pi;
    gt_v_radians = (GSVMeta(seq_id).pitch / 360) * 2 * pi;
    % viewpoints of GT
    gt_uv = [gt_u_radians, gt_v_radians];
    im_sz = size(gtImg);
    
    [ xyz_GT ] = uv2xyzN(gt_uv);

    % parameters
    im_width = im_sz(2);
    im_height = im_sz(1);

    pano_resolution_factor = round(360/gtFOV); %pano_resolution_factor = round(360/90);
    sphereW = im_width * pano_resolution_factor; % keep resolutions
    sphereH = sphereW/2;
    
%     if debug_flag
        h_im_GT = figure(5);
        imshow(uint8(gtImg));
        hold on
        title ('original image with sample bbox')
%     end
    % Conver nms coordinates to xyz
    nms_bbox_coors = nms_bboxes(:,1:8);
    nms_bbox_coors = reshape(nms_bbox_coors, 2,size(nms_bboxes,1)*4)';
    nms_bbox_uv  = coords2uv( nms_bbox_coors, sphereW, sphereH );
    nms_bbox_xyz = uv2xyzN( nms_bbox_uv );
    
    curr_pano_boxes = [nms_bboxes(ismember([nms_bboxes(:,10)],gtSeq),:)];
    curr_pers_boxes = [bboxes(ismember([bboxes.cam],gtSeq))];
    for i = 1:size(curr_pano_boxes,1)
        bbox_coors = curr_pano_boxes(i,1:8);
        bbox_coors = reshape(bbox_coors, 2,4)';

        bbox_uv  = coords2uv( bbox_coors, sphereW, sphereH );
        bbox_xyz = uv2xyzN( bbox_uv );
        
        [bbox_target_ima, valid, division] = projectPoint2SeparateView( bbox_xyz, xyz_GT, gtFOV_rad, im_sz(1), im_sz(2));

        % extract bounding rectangle on the other image
        xmin_other = min(bbox_target_ima(:,1));
        xmax_other = max(bbox_target_ima(:,1));
        ymin_other = min(bbox_target_ima(:,2));
        ymax_other = max(bbox_target_ima(:,2));
        
        xmin = curr_pers_boxes(i).x;
        xmax = xmin + curr_pers_boxes(i).width;
        ymin = curr_pers_boxes(i).y;
        ymax = ymin + curr_pers_boxes(i).height;

        % paint bbox and points
%         if debug_flag
            figure(5), rectangle('Position', [xmin_other, ymin_other, xmax_other-xmin_other,ymax_other-ymin_other],'EdgeColor','r', 'LineWidth', 3),...
                hold on, rectangle('Position', [xmin, ymin, xmax-xmin,ymax-ymin],'EdgeColor','y', 'LineWidth', 2)
            
%                 plot(bbox_target_ima(:,1),bbox_target_ima(:,2),'r+', 'MarkerSize', 4);
%         end
         
        
        
    end
    
end