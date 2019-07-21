function [bbox_target_ima theta] = alignBBox(bbox_target_ima, top)
%ALIGNBBOX Summary of this function goes here
%   Detailed explanation goes here
    debug_flag = 0;
    theta = 0;
%     x = bbox_target_ima(1:4,1);
%     y = bbox_target_ima(1:4,2);
%     K = convhull(x,y);
%     lbIdx = K(2);
%     rbIdx = K(3);
    if nargin < 2
        top = 0;
    end
    if( ~ top )
        bbox_target_ima(:,2) = -bbox_target_ima(:,2);
        lbIdx = 4;
        rbIdx = 3;
    else
        lbIdx = 1;
        rbIdx = 2;
    end
        

    if (abs(bbox_target_ima(rbIdx,2) - bbox_target_ima(lbIdx,2)) > 5)
        if(bbox_target_ima(rbIdx,2) > bbox_target_ima(lbIdx,2))
            v1 = [bbox_target_ima(rbIdx,1),bbox_target_ima(rbIdx,2)]-[bbox_target_ima(lbIdx,1),bbox_target_ima(lbIdx,2)];
            v2 = [bbox_target_ima(rbIdx,1),bbox_target_ima(lbIdx,2)]-[bbox_target_ima(lbIdx,1),bbox_target_ima(lbIdx,2)];
            phi = -acos(sum(v1.*v2)/(norm(v1)*norm(v2)));
        else
            v1 = [bbox_target_ima(lbIdx,1),bbox_target_ima(lbIdx,2)]-[bbox_target_ima(rbIdx,1),bbox_target_ima(rbIdx,2)];
            v2 = [bbox_target_ima(lbIdx,1),bbox_target_ima(rbIdx,2)]-[bbox_target_ima(rbIdx,1),bbox_target_ima(rbIdx,2)];
            phi = acos(sum(v1.*v2)/(norm(v1)*norm(v2)));
        end
        theta =  phi * 180/pi

        if (isnan(theta))
            return
        end

        polyin = polyshape(bbox_target_ima(1:4,1)',bbox_target_ima(1:4,2)');

        rot_pnt = [mean([min(bbox_target_ima(1:4,1)), max(bbox_target_ima(1:4,1))]), mean([min(bbox_target_ima(1:4,2)), max(bbox_target_ima(1:4,2))])];
        poly2 = rotate(polyin,theta,rot_pnt);

        if debug_flag == 1
            figure(7)
            hold off
            plot([polyin poly2])
            hold on
            line([bbox_target_ima(lbIdx,1),bbox_target_ima(rbIdx,1)], [bbox_target_ima(lbIdx,2),bbox_target_ima(rbIdx,2)], 'Color', 'yellow')
            line([bbox_target_ima(lbIdx,1),bbox_target_ima(lbIdx,1)], [bbox_target_ima(lbIdx,2),bbox_target_ima(rbIdx,2)], 'Color', 'yellow')
            set (gca, 'Color' , 'k' )
            axis equal
        end
        bbox_target_ima = poly2.Vertices;
%         bbox_target_ima(:,2) = - bbox_target_ima(:,2);
    end
    if(~top)
        bbox_target_ima(:,2) = - bbox_target_ima(:,2);
    end

end

