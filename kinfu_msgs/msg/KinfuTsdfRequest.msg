KinfuRequestHeader tsdf_header

# if true, the kinfu will be reset during extraction (faster)
# can be used with REQUEST_TYPE_PING to just reset the kinfu
bool request_reset

# if true, this affine transformation will be applied to the points during extraction
# this is NOT ALLOWED for REQUEST_TYPE_GET_TSDF
bool request_transformation
KinfuPose transformation

# If request_bounding_box, the mesh/cloud is cropped before publishing.
# bounding_box_min and bounding_box_max define a minimum and maximum value
# for each point coordinate, i.e. the bounding box.
# Only points/triangles inside the bounding box are published.
# The transformation (from request_transformation above) is applied before
# cropping.
# Note that this option is only useful to reduce ROS message size. In
# the current implementation, KinFu always processes the whole point cloud,
# even if only a part is needed.
bool request_bounding_box
KinfuCloudPoint bounding_box_min
KinfuCloudPoint bounding_box_max

# if true, only points inside this sphere will be used
# for now, implemented only for REQUEST_TYPE_GET_VOXEL_COUNT
bool request_sphere
KinfuCloudPoint sphere_center
float32 sphere_radius

# if true, the resulting point cloud will be subsampled
# currently implemented only for REQUEST_TYPE_GET_KNOWN
bool request_subsample
float32 subsample_voxel_size

# for REQUEST_TYPE_GET_VIEW, distance from the sensor to the center of the TSDF volume
float32 tsdf_center_distance
# accepted distance before shifting the TSDF volume (0 = default)
float32 shift_distance_threshold

# for REQUEST_TYPE_GET_VIEW, REQUEST_TYPE_GET_VIEW_CLOUD, REQUEST_TYPE_GET_VOXEL_COUNT
bool request_camera_intrinsics
KinfuCameraIntrinsics camera_intrinsics

bool request_view_poses
KinfuPose[] view_poses

# for view operations, ignore the points outside this bounding box
# useful to exclude noisy areas at the edges of the observed world
bool request_bounding_box_view
KinfuCloudPoint bounding_box_view_min
KinfuCloudPoint bounding_box_view_max

# for REQUEST_TYPE_GET_VIEW_SCORE
float32 occupied_weight
float32 occupied_distance_weight
float32 unknown_weight
float32 unknown_distance_weight

# remove duplicate points if REQUEST_TYPE_GET_CLOUD or REQUEST_TYPE_GET_MESH
bool request_remove_duplicates
