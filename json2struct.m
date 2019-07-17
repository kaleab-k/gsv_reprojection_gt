function [GSVMeta, YOLOResult] = json2struct(dir, subdir, yoloresult)

% dir = '../../dataset/40.4166718,-3.7032952/';
% subdir = 'M:DRIVING_S:608x608';
YOLOResult = [];
jpeg_folder = [dir subdir '-jpegs'];

if nargin > 2
    yolo = yoloresult;
else
    yolo = false;
end



if yolo
    yolo_fn = [dir '/yolov3/' subdir '_yolo.json'];
    YOLOResult = jsondecode(fileread(yolo_fn));
    seqNums = unique([YOLOResult.seqNumber]);
    images_yolo = {};


    for ii = 1:length(YOLOResult)
        seq_num = YOLOResult(ii).seqNumber;
        currentfilename = [jpeg_folder, '/' , sprintf('%05d.jpg',seq_num)];
        images_yolo{ii} = currentfilename;
    end

    images_yolo = images_yolo';
    [YOLOResult(:).dataSource] = images_yolo{:};
end

gsv_fn = [dir subdir '.json'];

GSVMeta = jsondecode(fileread(gsv_fn));
GSVMeta = GSVMeta.images;


% gsv_meta.path = [dir subdir];

for seq_num = 0:length(GSVMeta)-1
    currentfilename = [jpeg_folder, '/' , sprintf('%05d.jpg',seq_num)];
    images_gsv{seq_num+1} = currentfilename;
end
images_gsv = images_gsv';
[GSVMeta.dataSource] = images_gsv{:};
% save('yolo_result.mat', 'YOLOResult','GSVMeta');

end