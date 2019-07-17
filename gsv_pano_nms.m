function [pan, nms_bboxes] = gsv_pano_nms(pano_bboxes)
%GSV_PANO_NMS Summary of this function goes here
%   Detailed explanation goes here

pan = double(imread('panorama.png'));
crop = pan(:,1:238,:);
pan(:,1:238,:) = [];
pan = [pan crop];

% nms_bboxes = nms_oriented_rectangles_2(pano_bboxes, 500,1);
pick = nms_oriented_rectangles(pano_bboxes, 0.3);
nms_bboxes = pano_bboxes(pick,:);

end

