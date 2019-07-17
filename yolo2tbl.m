function [results] = yolo2tbl(YOLOResult, GTClassNames, maxSeqNum)
%YOLO2TBL converts YOLO detection Struct to Table
%   Detailed explanation goes here
    numImages = length(unique([maxSeqNum]));
    results = table('Size',[numImages 3],...
    'VariableTypes',{'cell','cell','cell'},...
    'VariableNames',{'Boxes','Scores','Labels'});
    
    classes = unique({YOLOResult.class});
    
    for seq_ii = 0:maxSeqNum %unique([YOLOResult.seqNumber])
        curr_seq_det = YOLOResult([YOLOResult.seqNumber] == seq_ii);
%         for class_idx=1:length(classes)
%             class_name = classes{class_idx};
%             class_inst = curr_seq_det(strcmp([curr_seq_det.class], class_name));
            gT_A = [];
            score_A = [];
            class_C = categorical([GTClassNames]);
            class_A = [];
            for ii = 1:length(curr_seq_det)
                if(any(strcmp(GTClassNames, curr_seq_det(ii).class)) && curr_seq_det(ii).width >0 && curr_seq_det(ii).height > 0)
                   bbox = [ curr_seq_det(ii).x curr_seq_det(ii).y curr_seq_det(ii).width curr_seq_det(ii).height ];
                   gT_A = [gT_A; bbox];
                   class_id = find(class_C==curr_seq_det(ii).class);
                   class_A = [class_A; class_C(class_id)];
                   score_A = [score_A; curr_seq_det(ii).confidence];
                end
            end
%             gT{seq_ii+1,class_idx} = gT_A;
            results.Boxes{seq_ii+1} = gT_A;
            results.Scores{seq_ii+1} = score_A;
            results.Labels{seq_ii+1} = class_A;
%         end
    end


    
        

end

