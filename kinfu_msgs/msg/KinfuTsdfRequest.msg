KinfuRequestHeader tsdf_header

# if true, the kinfu will be reset during extraction (faster)
# can be used with REQUEST_TYPE_PING to just reset the kinfu
bool request_reset

# if true, this affine transformation will be applied to the points during extraction
# this is NOT ALLOWED for REQUEST_TYPE_GET_TSDF
bool request_transformation
# the linear matrix, row major
float32[9] transformation_linear
# the translation, x y z
float32[3] transformation_translation
# new_point = linear * old_point + translation

# if true, points and/or triangles outside this bounding box will be removed
# this is applied after the transformation above
bool request_bounding_box
KinfuCloudPoint bounding_box_min
KinfuCloudPoint bounding_box_max

# if true, the resulting point cloud will be subsampled
# currently implemented only for REQUEST_TYPE_GET_KNOWN
bool request_subsample
float32 subsample_voxel_size
