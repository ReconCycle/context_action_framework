int8 id

# Label intEnum
int8 label

# segmentation score
float32 score

# non-oriented bounding box (meters) 
float32[] bb

# oriented bounding box (meters)
float32[] obb

# 3d oriented bounding box (meters)
float32[] obb_3d

# transform of object center
geometry_msgs/Transform tf