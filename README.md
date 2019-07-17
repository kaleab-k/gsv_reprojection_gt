# gsv_reprojectionGT
Initial tests for the creation of an object detection "ground truth" from the reprojection of detections in multiple images

# Easy Setup for the time being
## Directory Hierarchy 
- Create a directory named _matlab_, inside the **gsv_database_creator**
- Create another directory with the name of _panoramic_reprojection_ in the above directory _matlab_, put those scripts in this new directory
- Download PanoBasic from https://github.com/yindaz/PanoBasic and copy everything inside the folder _matlab_.

**P.S.** The given folder names are not mandatory.

## Experimentation
- For the ground truth construction and evaluate the YOLO results, please execute the _test_reprojection.m_.
  - I have included the rotation of the tilted rectangles. There is some improvement in decreasing the bounding box size but not much interms of overall mAP. Maybe, I haven't implemented it correctly. I would be grateful if you can suggest me a better way. 
- For the non-maxima suppression in tha panaroma image, please execute the _NMS.m_.



