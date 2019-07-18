function [GTruth] = struct2gt(GTruthStruct, GTMeta )

classes = unique([GTruthStruct.class]);

gT = {};

for seq_ii=0:max([GTMeta.seqNumber])
    images{seq_ii+1} = GTMeta(seq_ii+1).dataSource;
    curr_seq_det = GTruthStruct([GTruthStruct.seqNumber] == seq_ii);
    for class_idx=1:length(classes)
        class_name = classes{class_idx};
        class_inst = curr_seq_det(strcmp([curr_seq_det.class], string(class_name)));
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

% save("yolo.mat", 'GTruth','GTMeta');
end




