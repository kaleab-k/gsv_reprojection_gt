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
        
        [xmin, xmax, ymin, ymax] = minMaxXY(bbox_target_ima);

        if(sum(valid)==0)
            continue;
        elseif(~any(bbox_target_ima(1:4,1) >= 0 & bbox_target_ima(1:4,1) <= im_width | bbox_target_ima(1:4,2) >= 0 & bbox_target_ima(1:4,2) <= im_height) )
            continue;
        end
        
        if debug_flag == 1
            figure(5), plot(bbox_target_ima(:,1),bbox_target_ima(:,2),'y+', 'MarkerSize', 14), set (gca, 'Color' , 'k' );
        end
        
        [bbox_target_ima theta] = alignBBox(bbox_target_ima, 0);
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
        [xmin, xmax, ymin, ymax] = minMaxXY(bbox_target_ima);
        if( (xmax - xmin < 2 && (xmin < 5 || xmax > im_width -5) ) || (ymax - ymin < 2 && (ymin < 5 || ymax > im_width -5) ) )
            continue;
        elseif ( (xmax - xmin >= (im_width-10) ) || (ymax - ymin >= (im_height-10)) )
            continue;
        end
        
        % paint bbox and points
        if debug_flag == 1
            figure(5), rectangle('Position', [xmin, ymin, xmax-xmin,ymax-ymin],'EdgeColor','r', 'LineWidth', 2);
            figure(5), plot(bbox_target_ima(:,1),bbox_target_ima(:,2),'g+', 'MarkerSize', 10);
        end
         
        GTruthNMS(end+1).seqNumber = bboxes_xyz(i).cam;
        GTruthNMS(end).class = bboxes_xyz(i).class;
        GTruthNMS(end).x = xmin;
        GTruthNMS(end).y = ymin;
        GTruthNMS(end).width = xmax - xmin;
        GTruthNMS(end).height = ymax - ymin;
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

%======================4
% Extract MinMax
function [xmin, xmax, ymin, ymax] = minMaxXY(bbox)
    xmin = min(bbox(:,1));
    xmax = max(bbox(:,1));
    ymin = min(bbox(:,2));
    ymax = max(bbox(:,2));
end
    