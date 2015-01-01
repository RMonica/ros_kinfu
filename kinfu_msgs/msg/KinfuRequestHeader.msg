uint32 request_type
uint32 request_id
string request_source_name

uint32 REQUEST_TYPE_PING = 0
uint32 REQUEST_TYPE_GET_TSDF = 1
uint32 REQUEST_TYPE_GET_MESH = 2
uint32 REQUEST_TYPE_GET_CLOUD = 3
uint32 REQUEST_TYPE_GET_KNOWN = 4 # set the parameter extract_known_points to true to use this
