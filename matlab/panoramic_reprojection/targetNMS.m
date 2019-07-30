function [GTruthStruct] = targetNMS(GTruthNMS, GTruthStruct, seqNum, GSVImgSrc)
%TARGETNMS Summary of this function goes here
%   Detailed explanation goes here
    debug_flag  = 0;
    overlapThresh = 0.35;
    if (~isempty(GTruthNMS))
        [detectionResults cam_seq] = yolo2tbl(GTruthNMS, unique({GTruthNMS.class}), max([GTruthNMS.seqNumber]));
        Labels_A = [];
        for i = 1:length(detectionResults.Labels)
            Labels_A = [Labels_A; detectionResults.Labels{i,1}];
        end
        [selectedBboxes, selectedScores, selectedLabels] = selectStrongestBboxMC(cell2mat(detectionResults.Boxes),cell2mat(detectionResults.Scores), Labels_A, cell2mat(cam_seq(:)),  'OverlapThreshold', overlapThresh);
    %         [selectedBboxes, selectedScores, selectedLabels] = selectStrongestBboxMulticlass(cell2mat(detectionResults.Boxes),cell2mat(detectionResults.Scores), Labels_A,  'OverlapThreshold', overlapThresh);
        for i = 1:size(selectedBboxes,1)
            xmin_nms = selectedBboxes(i,1);
            ymin_nms = selectedBboxes(i,2);
            width_nms =  selectedBboxes(i,3);
            height_nms =  selectedBboxes(i,4);
            score = selectedScores(i,1);
            class = selectedLabels(i,1);

            %% Store to the final GTruthStruct
            GTruthStruct(end+1).seqNumber = seqNum;
            GTruthStruct(end).class = string(class);
            GTruthStruct(end).x = xmin_nms;
            GTruthStruct(end).y = ymin_nms;
            GTruthStruct(end).width = width_nms;
            GTruthStruct(end).height = height_nms;
            GTruthStruct(end).confidence = score;
            GTruthStruct(end).dataSource = GSVImgSrc;
            if debug_flag == 1
                figure(5), rectangle('Position', [xmin_nms+0.2, ymin_nms+0.2, width_nms, height_nms],'EdgeColor','g', 'LineWidth', 2);
                figure(6), rectangle('Position', [xmin_nms+0.2, ymin_nms+0.2, width_nms, height_nms],'EdgeColor','g', 'LineWidth', 2);
            end
        end
    end
end

