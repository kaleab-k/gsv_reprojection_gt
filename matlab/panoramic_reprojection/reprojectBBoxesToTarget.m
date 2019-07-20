function [GTruth GSVMeta] = reprojectBBoxesToTarget(dir,subdir, bboxes_xyz)

debug_flag = 0;

%% Load the GSV Metadata for the FOV=120
[GSVMeta YOLOResult] = json2struct(dir, subdir, true);

GTruthStruct = struct('seqNumber',{}, 'class',{},'x',{}, 'y',{}, 'width',{},'height',{},'confidence',{},'dataSource',{});

%% Reproject bboxes_xyz to target 
for gsvSeq = 0:max([GSVMeta.seqNumber])
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

    if debug_flag == 1
        h_im_GSV = figure(5);
        imshow(uint8(GSVImg));
        hold on
        title ('NMS on the bboxes')
%         hold off
        h_im_GSV_2 = figure(6);
        imshow(uint8(GSVImg));
        hold on
        title ('GT vs YOLOResult')
    end
    
    for i = 1:length(bboxes_xyz)
       
        bbox_xyz = bboxes_xyz(i).bbox_xyz;
        
        [bbox_target_ima, valid, division] = projectPoint2SeparateView( bbox_xyz, xyz_GSV, GSVFOV_rad, im_height, im_width);
        
        if(sum(valid)==0)
            continue;
        elseif(~any(bbox_target_ima(:,1) >= 0 & bbox_target_ima(:,1) <= im_width | bbox_target_ima(:,2) >= 0 & bbox_target_ima(:,2) <= im_height) )
            continue;
        end
        
        if debug_flag == 1
            figure(5), plot(bbox_target_ima(:,1),bbox_target_ima(:,2),'y+', 'MarkerSize', 14);
        end
        
        [bbox_target_ima theta] = alignBBox(bbox_target_ima);
        if (isnan(theta))
            continue;
        end
        if( isempty(bbox_target_ima) )
            continue;
        end
               
        bbox_target_ima(bbox_target_ima < 0) = 0;
        bbox_target_ima(bbox_target_ima(:,1) > im_width,1)=im_width;
        bbox_target_ima(bbox_target_ima(:,2) > im_height,2)=im_height;
        
       
        % extract bounding rectangle on the other image
        xmin_other = min(bbox_target_ima(:,1));
        xmax_other = max(bbox_target_ima(:,1));
        ymin_other = min(bbox_target_ima(:,2));
        ymax_other = max(bbox_target_ima(:,2));
        
        disp([gsvSeq i]);
       
         if( (xmax_other - xmin_other < 10) || (ymax_other - ymin_other < 10) )
            continue;
         elseif ( (xmax_other - xmin_other >= (im_width-5) ) || (ymax_other - ymin_other >= (im_height-5)) )
            continue;
         end
         
        % paint bbox and points
        if debug_flag == 1
            figure(5), rectangle('Position', [xmin_other, ymin_other, xmax_other-xmin_other,ymax_other-ymin_other],'EdgeColor','r', 'LineWidth', 2);
            figure(5), plot(bbox_target_ima(:,1),bbox_target_ima(:,2),'g+', 'MarkerSize', 10);
        end
         
        GTruthNMS(end+1).seqNumber = bboxes_xyz(i).cam;
        GTruthNMS(end).class = bboxes_xyz(i).class;
        GTruthNMS(end).x = xmin_other;
        GTruthNMS(end).y = ymin_other;
        GTruthNMS(end).width = xmax_other - xmin_other;
        GTruthNMS(end).height = ymax_other - ymin_other;
        GTruthNMS(end).confidence = bboxes_xyz(i).score;
        GTruthNMS(end).dataSource = GSVImgSrc;
       
    end
    
    %% Draw the YOLOResults just for observation
    if debug_flag == 1
        yolo_bboxes = YOLOResult(ismember([YOLOResult.seqNumber], gsvSeq));
        for j = 1:length(yolo_bboxes)
            bbox =  yolo_bboxes(j);
            figure(6), hold on, rectangle('Position', [bbox.x, bbox.y, bbox.width, bbox.height],'EdgeColor','b', 'LineWidth', 2);
        end
    end
    
    [GTruthStruct] = targetNMS(GTruthNMS, GTruthStruct, gsvSeq, GSVImgSrc);
    
end
GTruth =  struct2gt(GTruthStruct, GSVMeta);

end

