KinfuRequestHeader tsdf_header

# if true, the kinfu will be reset during extraction (faster)
# can be used with REQUEST_TYPE_PING to just reset the kinfu
bool request_reset

# if true, this affine transformation will be applied to the points during extraction
# this is NOT ALLOWED for REQUEST_TYPE_GET_TSDF
bool request_transformation
KinfuPose transformation

# if true, points and/or triangles outside this bounding box will be removed
# this is applied after the transformation above
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

# for view operations, treat all the points outside this bounding box as empty
# useful to exclude noisy areas at the edges of the observed world
bool request_bounding_box_view
KinfuCloudPoint bounding_box_view_min
KinfuCloudPoint bounding_box_view_max

# for REQUEST_TYPE_GET_VIEW, distance from the sensor to the center of the TSDF volume
float32 tsdf_center_distance

# for REQUEST_TYPE_GET_VIEW, uint32 REQUEST_TYPE_GET_VIEW_CLOUD, REQUEST_TYPE_GET_VOXEL_COUNT
bool request_camera_intrinsics
KinfuCameraIntrinsics camera_intrinsics

bool request_view_poses
KinfuPose[] view_poses
