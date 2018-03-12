uint32 command_type

# does nothing
uint32 COMMAND_TYPE_NOOP = 0
# resume a suspended kinfu
uint32 COMMAND_TYPE_RESUME = 1
# suspend a running kinfu
uint32 COMMAND_TYPE_SUSPEND = 2
# reset the kinfu. If has_pose_hint, the kinfu will be reset with that initial camera pose.
uint32 COMMAND_TYPE_RESET = 3
# enables or disables the minimum movement required to update the kinfu representation (using boolean_data[0])
uint32 COMMAND_TYPE_SET_ENABLED_MIN_MOVEMENT = 4
# enables or disables the tf frames used as forced hints (using boolean_data[0])
uint32 COMMAND_TYPE_SET_FORCED_TF_FRAMES = 5

# float_data must contain 4 values: [x,y,z,r], where [x,y,z] is the center and r is the radius
# the cleared volume is set to unknown by default.
# If boolean_data contains a single boolean set to true, the cleared volume is set to empty instead.
uint32 COMMAND_TYPE_CLEAR_SPHERE = 6
# float_data must contain 6 values: [mx,my,mz,Mx,My,Mz], where m[x,y,z] is the minimum and M[x,y,z] is the maximum
# WARNING: bounding box is aligned with the internal TSDF representation!
uint32 COMMAND_TYPE_CLEAR_BOUNDING_BOX = 7

# if kinfu is suspended, this executes it for a single frame
uint32 COMMAND_TYPE_TRIGGER = 8

# if (now > hint_expiration_time), the hint is ignored
time hint_expiration_time
# if the hint is forced, kinfu ICP is disabled for this iteration
bool hint_forced
# a rotation matrix, row major
float32[9] pose_hint_rotation
# the translation
float32[3] pose_hint_translation

bool[] boolean_data

float32[] float_data

string command_id
