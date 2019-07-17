function pick = nms_oriented_rectangles (boxes, overlap)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Title: Non-maximum Suppression for Oriented Bounding boxes
% Method: Separating Axis theorem to calculate Overlap between rectangles
% 'boxes' is a (:,9) vector with values of x1,y1, x2,y2, ... y4 and a score for each box. Every row in boxes vector holds the attributes of a bounding box
% 'overlap' is a threshold that denotes the ratio of the overlap area to the area of the bounding box
%
% Author: Vicky 
% vrkpt.india@gmail.com
% http://www.mathworks.com/matlabcentral/fileexchange/authors/274175
% http://vigneshramkrishnan.wordpress.com
% USING PAUL KOPROWSKI'S CODE [areaintersection.m] for calculating overlap area. Thank you Paul Koprowski !!!!
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
	windowWidth = 25;
	windowHeight = 75;
	
	boxArea = windowWidth * windowHeight;
	
	% Sort the scores vector
	s = boxes (:,end);
	[vals, I] = sort(s);
	pick = [];
	
	while ~isempty(I)
        last = length(I);
        i = I(last);
        pick = [pick; i];
        suppress = [last];
%         leftTopI = [boxes(i,1) boxes(i,2];
%         rightBottomI = [max(boxes(i,1:2:7)), max(boxes(i,2:2:8))];
        for pos = 1:last-1
            j = I(pos);
            
            windowWidth = max(boxes(i,1:2:7)) - min(boxes(i,1:2:7));
            windowHeight = max(boxes(i,2:2:8)) - min(boxes(i,2:2:8));

            boxArea = windowWidth * windowHeight;
			
			flag = 0;
			
            windowWidthJ = max(boxes(j,1:2:7)) - min(boxes(j,1:2:7));
            windowHeightJ = max(boxes(j,2:2:8)) - min(boxes(j,2:2:8));

            recA = [boxes(i,1), boxes(i,2), windowWidth, windowHeight];
            recB = [boxes(j,1), boxes(j,2), windowWidthJ, windowHeightJ];
            
			rectA (1,1) = boxes(i,1);
			rectA (1,2) = boxes(i,2);
			
			rectA (2,1) = boxes(i,3);
			rectA (2,2) = boxes(i,4);
			
			rectA (3,1) = boxes(i,5);
			rectA (3,2) = boxes(i,6);
			
			rectA (4,1) = boxes(i,7);
			rectA (4,2) = boxes(i,8);
			
			rectB (1,1) = boxes(j,1);
			rectB (1,2) = boxes(j,2);
			
			rectB (2,1) = boxes(j,3);
			rectB (2,2) = boxes(j,4);
			
			rectB (3,1) = boxes(j,5);
			rectB (3,2) = boxes(j,6);
			
			rectB (4,1) = boxes(j,7);
			rectB (4,2) = boxes(j,8);
			
			% Check if the two overlap
			flag = sat (rectA, rectB);
			
			% If there is overlap
            if flag == 1
				% Compute the overlap area
				areaOverlap = areaintersection(rectA, rectB, 50);
				overlapRatio = bboxOverlapRatio(recA, recB);
% 				if (areaOverlap/boxArea) > overlap
                if (overlapRatio) > overlap
					suppress = [suppress; pos];	
				end
			end
        end
        I(suppress) = [];
		
	end