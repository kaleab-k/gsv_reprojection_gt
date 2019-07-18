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
% load([ 'yolo_60.mat'])
% load('yolo_result_60.mat')

dir = '../../dataset/40.4166718,-3.7032952/';
subdir = 'M=DRIVING_S=608x608';

[GTruth, GTMeta] = build_gt(dir, subdir);
[GSVMeta, YOLOResult] = json2struct(dir, subdir, true);


% %% FIRST TRIAL with CARS 
% for gtSeq = 1:length(GTruth.DataSource.Source)
%     if (all(cellfun(@isempty,GTruth.LabelData{gtSeq,:}),2))
%         continue;
%     end
%     gtImgSrc = GTruth.DataSource.Source(gtSeq);
%     gtImg = double(imread(gtImgSrc{1}));
%     gtFOV = GTMeta(gtSeq).fov;
%     gtFOV_rad = (gtFOV/360)*2*pi; % fov in radians
%     % heading and pitch in uv format (radians)
%     gt_u_radians = ((GTMeta(gtSeq).heading-180) / 360) * 2 * pi;
%     gt_v_radians = (GTMeta(gtSeq).pitch / 360) * 2 * pi;
%     % viewpoints of GT
%     gt_uv = [gt_u_radians, gt_v_radians];
%     
%     seq_idx = [GSVMeta([GSVMeta.heading] == GTMeta(gtSeq).heading).seqNumber];
%     seq_idx = intersect(seq_idx, [YOLOResult.seqNumber]);
% 
% %     curr_seq_det = [YOLOResult(ismember([YOLOResult.seqNumber],seq_idx))];
%     bboxes = table2cell(GTruth.LabelData(gtSeq,:));
%     bbox_array = [];
%     class_array = {};
%     for bb_idx = 1:length(bboxes)
%         bbox_array(end+1:end+size(bboxes{1,bb_idx},1),1:4) = bboxes{1,bb_idx};
%         class_names = cell(size(bboxes{1,bb_idx},1),1);
%         class_names(:) = {GTruth.LabelDefinitions.Name{bb_idx,1}};
% %         [class_array{end+1:end+size(bboxes{1,bb_idx},1),1}] = {class_names{:}};
%         class_array = [class_array; string(class_names)];
%     end
% %     bbox_array2 = GTruth.LabelData.person{gtSeq,1};
% %     if (strcmp(fixProp,'fov'))
% %         curr_seq_det = [GSVMeta(ismember([GSVMeta.fov], GTMeta(gtSeq).fov))];
% %     elseif (strcmp(fixProp,'pitch'))
% %         curr_seq_det = [GSVMeta(ismember([GSVMeta.pitch], GTMeta(gtSeq).pitch))];
% %     end
%         curr_seq_det = [GSVMeta(ismember([GSVMeta.fov], GTMeta(gtSeq).fov) & ismember([GSVMeta.pitch], GTMeta(gtSeq).pitch) & ismember([GSVMeta.heading], GTMeta(gtSeq).heading))];
%     pano_bboxes = [];
%     pano_bboxes_2 = [];
%     for testIdx = 1:length(curr_seq_det)
% %         close all
%         YOLOImgSrc = curr_seq_det(testIdx).dataSource;
%         YOLOImg = double(imread(YOLOImgSrc));
%         YOLOMeta = curr_seq_det(testIdx);
%         YOLOFOV = YOLOMeta.fov;
%         % heading and pitch in uv format (radians)
%         YOLO_u_radians = ((YOLOMeta.heading-180) / 360) * 2 * pi;
%         YOLO_v_radians = (YOLOMeta.pitch / 360) * 2 * pi;
%         % viewpoints of GT
%         YOLO_uv = [YOLO_u_radians, YOLO_v_radians];
%         
%         im_sz = size(YOLOImg);
% 
%         % parameters
%         YOLOFOV_rad = (YOLOFOV/360)*2*pi; % fov in radians
%         im_width = im_sz(2);
%         im_height = im_sz(1);
% 
%         pano_resolution_factor = round(360/gtFOV); %pano_resolution_factor = round(360/90);
%         sphereW = im_width * pano_resolution_factor; % keep resolutions
%         sphereH = sphereW/2;
% 
%         if debug_flag
%             h_im_GT = figure(1);
%             imshow(uint8(gtImg));
%             hold on
%             title ('original image with sample bbox')
%             h_im_YOLO = figure(2);
%             imshow(uint8(YOLOImg));
%             hold on
%             title ('other pitch, no bbox information')
% 
%         end
% 
% 
%         % convert center of images in xyz format. check what this means exactly
%         [ xyz_GT ] = uv2xyzN(gt_uv);
%         [ xyz_YOLO ] = uv2xyzN(YOLO_uv);
% 
%         % projections of the images to panorama to see if they fit
%         [sphereImg_GT, validMap_GT] = im2Sphere(gtImg, gtFOV_rad, sphereW, sphereH, gt_u_radians, gt_v_radians );
%         [sphereImg_YOLO, validMap_YOLO] = im2Sphere(YOLOImg, YOLOFOV_rad, sphereW, sphereH, YOLO_u_radians, YOLO_v_radians );
% 
%         if debug_flag
%             h_pano = figure(3);
%             imshow(uint8(sphereImg_GT));
%             hold on
%             figure
%             imagesc(validMap_GT);
%         end
% 
%         if debug_flag
%             h_pano = figure(4);
%             imshow(uint8(sphereImg_YOLO));
%             hold on
%             figure
%             imagesc(validMap_YOLO);
%         end
%         % works well!!!
%         
%         % try with sample bounding boxes
%         % bbox_array = gTruth.LabelData.person{1,1};
%         for i = 1:size(bbox_array,1) % loop over the bboxes
% 
%             % bbox in another format
%             bbox.xmin = bbox_array(i,1);
%             bbox.ymin = bbox_array(i,2);
%             bbox.xmax = bbox.xmin + bbox_array(i,3);
%             bbox.ymax = bbox.ymin + bbox_array(i,4);
% 
%             bbx_points = [ (bbox.xmin) (bbox.xmax) (bbox.xmin) (bbox.xmax);
%                            (bbox.ymin) (bbox.ymin) (bbox.ymax) (bbox.ymax)]; 
%             bbx_points = bbx_points';
% 
%             % extract also upper and lower medium points (to avoid problems with big distorted objects)
%             upper_middle_x = round((bbox.xmin + bbox.xmax)/2);
%             upper_middle_y = bbox.ymin;
%             lower_middle_x = round((bbox.xmin + bbox.xmax)/2);
%             lower_middle_y = bbox.ymax;
%             bbx_points(5,:) = [upper_middle_x upper_middle_y];
%             bbx_points(6,:) = [lower_middle_x lower_middle_y];
% 
% 
%             % paint bbox 
%             if debug_flag
%                 figure(1),rectangle('Position', [bbox.xmin, bbox.ymin, bbox.xmax-bbox.xmin, bbox.ymax-bbox.ymin],	'EdgeColor','r', 'LineWidth', 3)
%             end
%             % paint bbox points 
%             if debug_flag
%                 figure(1), plot(bbx_points(:,1),bbx_points(:,2),'y+', 'MarkerSize', 4);
%             end
% 
%             % project bbox points to panorama image
%             [ bbox_xyz, out3DPlane ] = projectPointFromSeparateView(  bbx_points, xyz_GT, gtFOV_rad, im_sz(1), im_sz(2));
%             [ bbox_uv ] = xyz2uvN( bbox_xyz);
%             [ bbox_coords_pano ] = uv2coords( bbox_uv, sphereW, sphereH);
%             
%             pano_bboxes = [pano_bboxes; bbox_coords_pano ];
%             
%             if debug_flag
%                 figure(3), plot(bbox_coords_pano(:,1),bbox_coords_pano(:,2),'r+', 'MarkerSize', 4);
%             end
% 
%         %     % bbox image coords in the panorama to xyz format
%         %     [ bbox_uv ] = coords2uv( bbox_coords_pano, sphereW, sphereH );
%         %     [ xyz_bbox ] = uv2xyzN( bbox_uv );
% 
%             % reproject the bbox to the other image, pitch = -20�
%             [bbox_target_ima, valid, division] = projectPoint2SeparateView( bbox_xyz, xyz_YOLO, YOLOFOV_rad, im_sz(1), im_sz(2));
% 
%             % extract bounding rectangle on the other image
%             xmin_other = min(bbox_target_ima(:,1));
%             xmax_other = max(bbox_target_ima(:,1));
%             ymin_other = min(bbox_target_ima(:,2));
%             ymax_other = max(bbox_target_ima(:,2));
%             
%             % paint bbox and points
%             if debug_flag
%                 figure(2), rectangle('Position', [xmin_other, ymin_other, xmax_other-xmin_other,ymax_other-ymin_other],'EdgeColor','r', 'LineWidth', 3)
% %                 plot(bbox_target_ima(:,1),bbox_target_ima(:,2),'r+', 'MarkerSize', 4);
%             end
%          
%             
%             % store bbox
% %             gtr_idx = testIdx + (i-1);
%             class_names = GTruth.LabelDefinitions.Name;
%             if ( xmin_other >= 0 && ymin_other>= 0 && xmax_other <= im_width && ymax_other <= im_height )
%                 if(exist('GTReprojected') == 1)
%                     GTReprojected(end+1).seqNumber = YOLOMeta.seqNumber;  
%                 else
%                     GTReprojected(1).seqNumber = YOLOMeta.seqNumber;
%                 end
%                 GTReprojected(end).dataSource = YOLOMeta.dataSource;
%                 GTReprojected(end).x = xmin_other;
%                 GTReprojected(end).y = ymin_other;
%                 GTReprojected(end).width = xmax_other-xmin_other;
%                 GTReprojected(end).height = ymax_other-ymin_other;
%                 GTReprojected(end).class = class_array(i);
%             end
%             % double-check. Reproject back to the panoramic image
%                 % project bbox points to panorama image
%             [ bbox_target_xyz, out3DPlane ] = projectPointFromSeparateView(  bbox_target_ima, xyz_YOLO, YOLOFOV_rad, im_sz(1), im_sz(2));
%             [ bbox_target_uv ] = xyz2uvN( bbox_target_xyz);
%             [ bbox_target_pano ] = uv2coords( bbox_target_uv, sphereW, sphereH);
%             if debug_flag
%                 figure(4), plot(bbox_target_pano(:,1),bbox_target_pano(:,2),'r+', 'MarkerSize', 4);
%             end
% 
%             pano_bboxes_2 = [pano_bboxes_2; bbox_target_pano ];
% 
%         end
% 
%         % There is distortion present in the x axis. Probably due to the wide fov.
%         % If this is reduced, probably distortion will be lower    
% 
%         
%     end
%     
% end

%% EVALUATION
% groundTruthData = struct2tbl(GTReprojected);
groundTruthData = GTruth.LabelData;

gt_class_names = [fieldnames(groundTruthData)];
detectionResults = yolo2tbl(YOLOResult, gt_class_names(1:end-3), max([GTMeta.seqNumber]));

[averagePrecision,recall,precision] = evaluateDetectionPrecision(detectionResults,groundTruthData,0.1);
mAP = mean(averagePrecision)

% Plot precision/recall curve
for i = 1:length(averagePrecision)
    figure
    rec = recall(i);
    prec = precision(i);
    plot(rec{:},prec{:})
    xlabel('Recall')
    ylabel('Precision')
    grid on
    title(sprintf('%s: Average Precision = %.2f', string(GTruth.LabelDefinitions.Name(i)),averagePrecision(i)));
end
