int8 id

# transform start of levering
geometry_msgs/Transform from_tf

# transform end of levering
geometry_msgs/Transform to_tf

# on which detection does the levering start
context_action_framework/Detection from_det

# on/in which detection does the levering end
context_action_framework/Detection to_det

# oriented bounding box of gap (meters)
float32[] obb

# 3d oriented bounding box of gap (meters)
float32[] obb_3d