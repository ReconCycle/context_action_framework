int64 id                        # index in detections list
int64 tracking_id               # unique ID per label that is stable across frames.

int8 label                      # Label intEnum, hca/smoke_detector/battery/internals/...
int8 label_face                 # LabelFace intEnum front/back/side1/side2
string label_precise            # 01/01.1/03.1/...
string label_precise_name       # kalo/minal/fumonic/siemens/...
float32 score                   # segmentation score

# in pixel coords
geometry_msgs/Transform tf_px   # transform of object center
float32[] box_px                # non-oriented bounding box
float32[] obb_px                # oriented bounding box
float32[] center_px
float32[] polygon_px

# in world coords (meters)
geometry_msgs/Transform tf
float32[] box
float32[] obb
float32[] center
float32[] polygon

# in world coords the cuboid containing the object
float32[] obb_3d

string parent_frame
string table_name
string tf_name