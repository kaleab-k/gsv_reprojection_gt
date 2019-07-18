# gsv_reprojectionGT
Initial tests for the creation of an object detection "ground truth" from the reprojection of detections in multiple images

# Easy Setup for the time being
## Directory Hierarchy 
- Download PanoBasic from https://github.com/yindaz/PanoBasic and copy everything inside the folder _matlab_.

## Experimentation
- For the ground truth construction and evaluating the YOLO results, please execute the _test_reprojection.m_.  
- For the non-maxima suppression in tha panaroma image, please execute the _NMS.m_.

**Inquiry:**
- I have included the rotation of the tilted rectangles. There is some improvement in decreasing the bounding box size but not much interms of overall mAP. Maybe, I haven't implemented it correctly. I would be grateful if you can suggest me a better way. 


