function [GTruth] = struct2tbl(GTSruct)
%STRUCT2GT converts struct into groundtruth class
%   Detailed explanation goes here
    classes = unique([GTSruct.class]);
    ldc = labelDefinitionCreator();
    labelNames = cell(1,length(classes));
    gT = {};
    for seq_ii = unique([GTSruct.seqNumber])
        curr_seq_det = GTSruct([GTSruct.seqNumber] == seq_ii);
        for class_idx=1:length(classes)
            class_name = classes{class_idx};
            class_inst = curr_seq_det(strcmp([curr_seq_det.class], class_name));
            gT_A = zeros(length(class_inst),4);

            for ii = 1:length(class_inst)
               gT_A(ii,:) = [ class_inst(ii).x class_inst(ii).y class_inst(ii).width class_inst(ii).height ];
            end
            gT{seq_ii+1,class_idx} = gT_A;
        end
    end

    for class_idx=1:length(classes)
        class_name = classes{class_idx};
        class_name(isspace(class_name)) = [];
        addLabel(ldc,class_name,labelType.Rectangle);
        labelNames{1, class_idx} = [class_name];
    end
    labelDefs = create(ldc);

    GTruth = array2table(gT,'VariableNames',labelNames);
    
end

