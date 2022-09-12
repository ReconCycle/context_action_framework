int8 id
int8 tracking_id

int8 label    # Label intEnum
float32 score # segmentation score

# in pixel coords
geometry_msgs/Transform tf_px   # transform of object center
float32[] box_px                # non-oriented bounding box
float32[] obb_px                # oriented bounding box

# in world coords (meters)
geometry_msgs/Transform tf
float32[] box
float32[] obb
float32[] obb_3d

# polygon in pixel coords
float32[] polygon_px
