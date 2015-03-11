uint32 request_type
uint32 request_id
string request_source_name

uint32 REQUEST_TYPE_PING = 0
uint32 REQUEST_TYPE_GET_TSDF = 1
uint32 REQUEST_TYPE_GET_MESH = 2
uint32 REQUEST_TYPE_GET_CLOUD = 3
uint32 REQUEST_TYPE_GET_KNOWN = 4 # set the parameter extract_known_points to true to use this
uint32 REQUEST_TYPE_GET_VIEW = 5 # simulate a kinect view from a specific pose (defaults to current)
uint32 REQUEST_TYPE_GET_VIEW_CLOUD = 6 # simulate a kinect view from a specific pose and
                                       # returns a pointcloud with knowledge information (0: unknown, 1 occupied)
uint32 REQUEST_TYPE_GET_VOXEL_COUNT = 7 # returns an array of uint64 pairs (occupied count,unknown count)
                                        # for each view. If no views are specified, only one global pair is returned
uint32 REQUEST_TYPE_GET_VOXELGRID = 8 # returns a 3D array of float values. A bounding box is mandatory.
