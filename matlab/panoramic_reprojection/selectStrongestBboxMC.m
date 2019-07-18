%selectStrongestBboxMulticlass Select strongest multiclass bounding boxes from overlapping clusters.
%   selectedBboxes = selectStrongestBboxMulticlass(bboxes,scores,labels) 
%   returns selected bounding boxes that have a high confidence score. The
%   function uses greedy non-maximal suppression (NMS) to eliminate
%   overlapping bounding boxes only if they have the same class label. The
%   selected boxes are returned in selectedBboxes. 
%
%   Inputs
%   ------
%   bboxes - an M-by-4 matrix defining M bounding boxes. Each row contains
%            a 4-element vector [x y width height], where x and y specify
%            the upper-left corner of a bounding box.
%  
%   scores - an M-by-1 vector of scores corresponding to the input bounding
%            boxes.
%  
%   labels - is an M-by-1 vector of categorical or numeric labels
%            corresponding to the input bounding boxes.
%
%   [..., selectedScores, selectedLabels, index] = selectStrongestBboxMulticlass(...) 
%   additionally returns the scores, labels, and index associated with the 
%   selected bounding boxes.
%
%   [...] = selectStrongestBboxMulticlass(..., Name, Value) specifies
%   additional name-value pairs described below:
%
%   'RatioType'         A string, 'Union' or 'Min', specifying the
%                       denominator of bounding box overlap ratio. 
%                       See bboxOverlapRatio for detailed explanation of 
%                       the ratio definition.
%
%                       Default: 'Union'
%                       
%   'OverlapThreshold'  A scalar from 0 to 1. All bounding boxes around a
%                       reference box are removed if their overlap ratio 
%                       is above this threshold.
%
%                       Default: 0.5
%
%  Class Support 
%  ------------- 
%  bboxes, scores, and labels must be real, finite, and nonsparse. They can
%  be uint8, int8, uint16, int16, uint32, int32, single or double. In
%  addition, labels can be categorical. OverlapThreshold can be single or
%  double. Class of index output is double. Class of selectedBboxes is the
%  same as that of bbox input. Class of selectedScores is the same as that
%  of score input. Class of selectedLabels is the same as that of labels
%  input.
%
%  Example 
%  ------- 
%  % Create detectors using two different models. These will be used to
%  % generate multiclass detection results.
%  detectorInria = peopleDetectorACF('inria-100x41' );
%  detectorCaltech = peopleDetectorACF('caltech-50x21');
% 
%  % Apply the detectors.
%  I = imread('visionteam1.jpg');
%  [bboxesInria, scoresInria] = detect(detectorInria,I,'SelectStrongest',false);
%  [bboxesCaltech, scoresCaltech] = detect(detectorCaltech,I,'SelectStrongest',false);
% 
%  % Create categorical labels for each the result of each detector.
%  labelsInria = repelem("inria",numel(scoresInria),1);
%  labelsInria = categorical(labelsInria,{'inria','caltech'});
%  labelsCaltech = repelem("caltech",numel(scoresCaltech),1);
%  labelsCaltech = categorical(labelsCaltech,{'inria','caltech'});
% 
%  % Combine results from all detectors to for multiclass detection results.
%  allBBoxes = [bboxesInria;bboxesCaltech];
%  allScores = [scoresInria;scoresCaltech];
%  allLabels = [labelsInria;labelsCaltech];
% 
%  % Run multiclass non-maximal suppression
%  [bboxes, scores, labels] = selectStrongestBboxMulticlass(...
%       allBBoxes,allScores,allLabels,...
%       'RatioType','Min','OverlapThreshold',0.65);
% 
%  % Annotate detected people
%  annotations = string(labels) + ": " + string(scores);
%  I = insertObjectAnnotation(I, 'rectangle', bboxes, cellstr(annotations));
%  figure
%  imshow(I)
%  title('Detected people, scores, and labels')
%
%  See also bboxOverlapRatio, selectStrongestBbox.

% Copyright 2017-2018 The MathWorks, Inc.

function [selectedBbox, selectedScore, selectedLabel, index] = ...
                selectStrongestBboxMC(bbox, score, label, cam, varargin)
            
 
%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

nargoutchk(0,4)

isUsingCodeGeneration = ~isempty(coder.target);
% Parse and check inputs
if isUsingCodeGeneration
    [ratioType, overlapThreshold] = vision.internal.detector.selectStrongestValidation.validateAndParseOptInputsCodegen(mfilename,varargin{:});
    checkInputBboxScoreLabelCodegen(bbox, score, label);
else
    [ratioType, overlapThreshold] = vision.internal.detector.selectStrongestValidation.validateAndParseOptInputs(mfilename,varargin{:});
    checkInputBboxScoreAndLabel(bbox, score, label);
end

if isempty(bbox)
    selectedBbox = bbox;
    selectedScore = score;
    selectedLabel = label;
    index = [];  
    return;
end

if strncmpi(ratioType, 'Union', 1)
    isDivByUnion = true;
else
    isDivByUnion = false;
end

if ~isfloat(bbox)
    inputBbox = single(bbox);
else
    inputBbox = bbox;
end

% sort the bbox according to the score
[~, ind] = sort(score, 'descend'); 
inputBbox = inputBbox(ind, :);

inputLabel = cast(label(ind,:),'like',inputBbox);
inputCam = cam(ind,:);%cast(cam(ind,:),'like',inputBbox);

% isUsingCodeGeneration = 1;

if isUsingCodeGeneration
    selectedIndex = bboxOverlapSuppressionCodegen(inputBbox, inputLabel, inputCam, ...
        overlapThreshold, isDivByUnion);
else
    [~, selectedIndex] = visionBboxOverlapSuppression(inputBbox, ...
        overlapThreshold, isDivByUnion, inputLabel); 
end

index = sort(ind(selectedIndex), 'ascend');
selectedBbox = bbox(index, :);
selectedScore = score(index);
selectedLabel = label(index);

end

%==========================================================================
function checkInputBboxScoreAndLabel(bbox,score,label)
vision.internal.detector.selectStrongestValidation.checkInputBboxAndScore(bbox, score, mfilename);
checkLabel(label)
if ~isempty(label) && (numel(label) ~= size(bbox,1))    
     error(message('vision:visionlib:unmatchedBboxAndLabel'));
end
end

%========================================================================== 
function checkLabel(value)
if isempty(coder.target)
    if iscategorical(value)
        % categorical not supported for codegen.
        validateattributes(value,{'categorical'}, {'size',[NaN, 1]}, ...
            mfilename, 'label', 3);
    else
        validateattributes(value,{'uint8', 'int8', 'uint16', 'int16', 'uint32', ...
            'int32', 'double', 'single'}, {'real','nonsparse','finite','integer','size',[NaN, 1]}, ...
            mfilename, 'label', 3);
    end
else
    validateattributes(value,{'uint8', 'int8', 'uint16', 'int16', 'uint32', ...
        'int32', 'double', 'single'}, {'real','nonsparse','finite','integer','size',[NaN, 1]}, ...
        mfilename, 'label', 3);
end
end

%==========================================================================
function checkInputBboxScoreLabelCodegen(bbox, score,label)
% Validate the input box, score, and label
vision.internal.detector.selectStrongestValidation.checkInputBboxAndScoreCodegen(bbox, score, mfilename);      
checkLabel(label);                    
coder.internal.errorIf(~isempty(label) && (size(label,1) ~= size(bbox,1)), ...
                        'vision:visionlib:unmatchedBboxAndLabel');
end

%==========================================================================
function selectedIndex = bboxOverlapSuppressionCodegen(inputBbox, inputLabel, inputCam, ...
    overlapThreshold, isDivByUnion)

isKept = true(size(inputBbox,1), 1);

area = inputBbox(:,3).*inputBbox(:,4);
x1 = inputBbox(:,1);
x2 = inputBbox(:,1)+inputBbox(:,3); 
y1 = inputBbox(:,2); 
y2 = inputBbox(:,2)+inputBbox(:,4);

% for each bbox i, suppress all surrounded bbox j where j>i and overlap
% ratio is larger than overlapThreshold
numOfBbox = size(inputBbox,1);
for i = 1:numOfBbox 
    if ~isKept(i) 
        continue; 
    end
    
    label = inputLabel(i);   
    cam = inputCam(i);
    for j = (i+1):numOfBbox 
        if ~isKept(j)
            continue; 
        end
        
        if inputLabel(j) ~= label
            continue
        end
        
        if inputCam(j) == cam
            continue
        end

        % compute the intersect box
        width = min(x2(i), x2(j)) - max(x1(i), x1(j)); 
        if width <= 0 
            continue; 
        end

        height = min(y2(i), y2(j)) - max(y1(i), y1(j)); 
        if height <= 0 
            continue; 
        end

        areaOfIntersect = width * height; 
        if isDivByUnion 
            overlapRatio = areaOfIntersect/(area(i)+area(j)-areaOfIntersect); 
        else
            overlapRatio = areaOfIntersect/min(area(i), area(j)); 
        end

        if overlapRatio > overlapThreshold 
            isKept(j) = false; 
        end
    end
end

selectedIndex = find(isKept); 
end           
            
