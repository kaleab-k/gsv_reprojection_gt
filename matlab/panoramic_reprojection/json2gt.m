function [GTruth, GTMeta] = json2gt(dir, subdir)
% dir = '../../dataset/40.4166718,-3.7032952/';
% subdir = 'M:DRIVING_S:608x608';

yolo_fn = [dir 'yolov3/' subdir '_yolo.json'];
gsv_fn = [dir subdir '.json'];

gsv_meta = jsondecode(fileread(gsv_fn));
yolo_det = jsondecode(fileread(yolo_fn));

classes = unique({yolo_det.class});

jpeg_folder = [ dir subdir '-jpegs'];
 
seqNums = unique([yolo_det.seqNumber]);
images = {};
gT = {};

for seq_ii=0:length(gsv_meta.images)-1
    currentfilename = [jpeg_folder, '/' , sprintf('%05d.jpg',seq_ii)];
    images{seq_ii+1} = currentfilename;
    curr_seq_det = yolo_det([yolo_det.seqNumber] == seq_ii);
    for class_idx=1:length(classes)
        class_name = classes{class_idx};
        class_inst = curr_seq_det(strcmp({curr_seq_det.class}, class_name));
        gT_A = zeros(length(class_inst),4);
        
        for ii = 1:length(class_inst)
           gT_A(ii,:) = [ class_inst(ii).x class_inst(ii).y class_inst(ii).width class_inst(ii).height ];
        end
        gT{seq_ii+1,class_idx} = gT_A;
    end
end


gtSource = groundTruthDataSource(images);
% defs = table({'Cars'}, ...
% 	[labelType.Rectangle], ...
% 	'VariableNames',{'Name','Type'});
ldc = labelDefinitionCreator();
labelNames = cell(1,length(classes));
for class_idx=1:length(classes)
        class_name = classes{class_idx};
        class_name(isspace(class_name)) = [];
        addLabel(ldc,class_name,labelType.Rectangle);
        labelNames{1, class_idx} = [class_name];
end
labelDefs = create(ldc);

labelData = array2table(gT,'VariableNames',labelNames);

GTruth = groundTruth(gtSource,labelDefs,labelData);

GTMeta = gsv_meta.images;

% save("yolo.mat", 'GTruth','GTMeta');
end




