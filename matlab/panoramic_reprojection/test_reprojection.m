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

%% Set directory and sub-directory path 
dir = '../../dataset/40.4166718,-3.7032952/';
subdir = 'M=DRIVING_S=608x608';

%% Fetch BBoxes from all GT images  
bboxes_xyz = getBBoxesFromGT(dir, subdir);

%% Reproject BBoxes to the target images to generate a GT
[GTruth GTMeta] = reprojectBBoxesToTarget(dir,subdir, bboxes_xyz);

%% Read YOLO detections of target images
[GSVMeta, YOLOResult] = json2struct(dir, subdir, true);


%% EVALUATION

groundTruthData = GTruth.LabelData;

gt_class_names = [fieldnames(groundTruthData)];
detectionResults = yolo2tbl(YOLOResult, gt_class_names(1:end-3), max([GTMeta.seqNumber]));

[averagePrecision,recall,precision] = evaluateDetectionPrecision(detectionResults,groundTruthData,0.4);
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
