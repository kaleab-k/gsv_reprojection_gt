% Simple test. Same location, two images, (1) pitch = 0 and (2) pitch = -20.
% Test: take a bounding box in (1) and try to reproject it to image (2).
% Works!
clc
clear
close all
debug_flag = 0;
IoUThresh = 0.4;

%%%%%%%%%%%%%%%%% path operations
% add Pano2Context and VOC paths
% addpath('/PanoBasic');
cd('../PanoBasic')
add_path; % Pano2Context functions
cd('../panoramic_reprojection')
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

[averagePrecision,recall,precision] = evaluateDetectionPrecision(detectionResults,groundTruthData, IoUThresh);
mAP = mean(averagePrecision);
classes = GTruth.LabelDefinitions.Name;

load('Results','Results');
disp(GSVMeta(1).fov)
idx = find([Results(:).fov] == GSVMeta(1).fov)
if isempty(idx)
    Results(end+1).averagePrecision = averagePrecision;
    Results(end).recall = recall;
    Results(end).precision = precision;
    Results(end).classes = classes;
    Results(end).fov = GSVMeta(1).fov;
else
    Results(idx).averagePrecision = averagePrecision;
    Results(idx).recall = recall;
    Results(idx).precision = precision;
    Results(idx).classes = classes;
    Results(idx).fov = GSVMeta(1).fov;
end
save('Results','Results');
disp(Results);

evaluateResult;
