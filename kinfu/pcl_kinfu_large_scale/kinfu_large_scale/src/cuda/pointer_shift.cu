#include <pcl/gpu/kinfu_large_scale/tsdf_buffer.h>

template <typename T>
__device__ __forceinline__ void
shift_tsdf_pointer(T ** value, const pcl::gpu::kinfuLS::tsdf_buffer & buffer)
{
  ///Shift the pointer by (@origin - @start)
  *value += (buffer.tsdf_rolling_buff_origin - buffer.tsdf_memory_start);
  
  ///If we land outside of the memory, make sure to "modulo" the new value
  if(*value > buffer.tsdf_memory_end)
  {
    *value -= (buffer.tsdf_memory_end - buffer.tsdf_memory_start + 1); /// correction of bug found my qianyizh
  }       
}
