#ifndef TEMPLATED_EXTRACT_CUH
#define TEMPLATED_EXTRACT_CUH

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      __device__ int global_count_subtraction = 0;
      __device__ int overestimated_global_count = 0;

      struct FullScan6
      {
        ////////////////////////////////////////////////////////////////////////////////////////
        ///// Full Volume Scan6
        enum
        {
          CTA_SIZE_X = 32,
          CTA_SIZE_Y = 4,
          CTA_SIZE = CTA_SIZE_X * CTA_SIZE_Y,
          LOG_CTA_SIZE = (5 + 2),
        };

        PtrStep<short2> volume;

        mutable PtrSz<PointType> output_xyz;
        mutable PtrSz<PointType> output_normals;
        mutable PtrSz<float> output_intensity;

        mutable PtrStep<int> data_transfer_completion_matrix;

        pcl::gpu::kinfuLS::tsdf_buffer rolling_buffer;
        float3 cell_size;

        __device__ __forceinline__ void
        my_shift_tsdf_pointer(short2 ** value) const
        {
          *value += (rolling_buffer.tsdf_rolling_buff_origin - rolling_buffer.tsdf_memory_start);

          if(*value > rolling_buffer.tsdf_memory_end)
          {
            *value -= (rolling_buffer.tsdf_memory_end - rolling_buffer.tsdf_memory_start + 1);
          }
        }

        __device__ __forceinline__ float
        fetch (int x, int y, int z, int& weight) const
        {
          float tsdf;
          const short2* tmp_pos = &(volume.ptr (rolling_buffer.voxels_size.y * z + y)[x]);
          short2* pos = const_cast<short2*> (tmp_pos);
          unpack_tsdf (*pos, tsdf, weight);

          return tsdf;
        }

        __device__ __forceinline__ float
        fetch_with_rolling_buffer (int x, int y, int z, int& weight) const
        {
          float tsdf;
          const short2* tmp_pos = &(volume.ptr (rolling_buffer.voxels_size.y * z + y)[x]);
          short2* pos = const_cast<short2*> (tmp_pos);

          my_shift_tsdf_pointer (&pos);

          unpack_tsdf (*pos, tsdf, weight);

          return tsdf;
        }

        template <typename EXTRACTOR>
        __device__ __forceinline__ void
        templatedExtract (EXTRACTOR extractor) const
        {
          int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
          int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

          const int ftid = Block::flattenedThreadId ();
          __shared__ int cta_buffer[CTA_SIZE];

          int maximum_Z = rolling_buffer.voxels_size.z - EXTRACTOR::MIN_Z_MARGIN;
          // find min valid z for this thread
          bool valid_xy = (x >= EXTRACTOR::MIN_X_MARGIN) && (y >= EXTRACTOR::MIN_Y_MARGIN) &&
            (x < rolling_buffer.voxels_size.x - EXTRACTOR::MIN_X_MARGIN) && (y < rolling_buffer.voxels_size.y - EXTRACTOR::MIN_Y_MARGIN);
          // fetch minimum z and distribute it to all threads
          if (ftid == 0)
            cta_buffer[0] = max(data_transfer_completion_matrix.ptr(blockIdx.y)[blockIdx.x], EXTRACTOR::MIN_Z_MARGIN);
          __syncthreads();
          int minimum_Z = cta_buffer[0];

          int z;
          for (z = minimum_Z; z < maximum_Z; ++z)
          {
            int local_count = 0;

            if (valid_xy)
            {
              local_count = extractor.filter(*this,rolling_buffer,x,y,z);
            }

            // local_count counts the number of zero crossing for the current thread.
            // Now we need to merge this knowledge with the other threads
            // not we fulfilled points array at current iteration
            int total_block_points = __syncthreads_count (local_count > 0);

            if (total_block_points > 0)  ///more than 0 zero-crossings
            {
              // Compute offset of current thread
              // Call in place scanning (see http://http.developer.nvidia.com/GPUGems3/gpugems3_ch39.html)
              cta_buffer[ftid] = local_count;
              __syncthreads();

              #pragma unroll
              for (int i = 0; i < LOG_CTA_SIZE; ++i)
              {
                int t_offset = (1 << i);
                int other_value;
                if (ftid >= t_offset)
                  other_value = cta_buffer[ftid - t_offset];
                __syncthreads();
                if (ftid >= t_offset)
                  cta_buffer[ftid] = cta_buffer[ftid] + other_value;
                __syncthreads();
              }
              int offset = ftid > 0 ? cta_buffer[ftid - 1] : 0; //How many crossings did we have before this thread ?
              total_block_points = cta_buffer[CTA_SIZE - 1];    // update total_block_points with the actual value

              __syncthreads();
              // We want to do only 1 operation per block -> because it is faster
              if (ftid == 0)
              {
                // We use atomicAdd, so that threads do not collide
                int old_global_count = atomicAdd (&overestimated_global_count, total_block_points);
                cta_buffer[0] = old_global_count;
              }
              __syncthreads();
              int old_global_count = cta_buffer[0];

              // Sanity check to make sure our output_xyz buffer is not full already
              bool full = extractor.isFull (*this, old_global_count + total_block_points);

              if (full)
              {
                if (ftid == 0)
                  atomicAdd (&global_count_subtraction, total_block_points);
                break;
              }

              __syncthreads();

              // Perform compaction (dump all current crossings)
              for (int l = 0; l < local_count; ++l)
              {
                int offset_storage = old_global_count + offset + l;
                extractor.store (*this,offset_storage,l);
              }

              __syncthreads();
            }

          } /* for(int z = 0; z < VOLUME_Z - 1; ++z) */

          // Save the z, so we can restart next execution
          if (ftid == 0)
            data_transfer_completion_matrix.ptr(blockIdx.y)[blockIdx.x] = z;

        } // templated_extract

        __device__ __forceinline__ void
        store_point_type (float x, float y, float z, float4* ptr, int offset) const
        {
          *(ptr + offset) = make_float4 (x, y, z, 0);
        }

        //INLINE FUNCTION THAT STORES XYZ AND INTENSITY VALUES IN 2 SEPARATE DeviceArrays.
        // ptr_xyz: pointer to the BEGINNING of the XYZ deviceArray
        // ptr_instensity: pointer to the BEGINNING of the Intensity deviceArray
        // offset: offset to apply to both XYZ and Intensity
        __device__ __forceinline__ void
        store_point_intensity (float x, float y, float z, float i, float4* ptr_xyz, float* ptr_intensity, int offset) const
        {
          *(ptr_xyz + offset) = make_float4 (x, y, z, 0);
          *(ptr_intensity + offset) = i;
        }

        __device__ __forceinline__ void
        store_point_normals (float x, float y, float z, float nx, float ny, float nz,
                             float4* ptr_xyz, float4* ptr_normals, int offset) const
        {
          *(ptr_xyz + offset) = make_float4 (x, y, z, 0);
          *(ptr_normals + offset) = make_float4 (nx, ny, nz, 0);
        }

        __device__ __forceinline__ void
        store_point_normals_curvature (float x, float y, float z, float nx, float ny, float nz, float c,
                             float4* ptr_xyz, float4* ptr_normals, int offset) const
        {
          *(ptr_xyz + offset) = make_float4 (x, y, z, 0);
          *(ptr_normals + offset) = make_float4 (nx, ny, nz, c);
        }

        __device__ __forceinline__ void
        store_point_type (float x, float y, float z, float3* ptr, int offset) const
        {
          *(ptr + offset) = make_float3 (x, y, z);
        }

        static int get_result_size(int & data_transfer_finished)
        {
          int overestimated,subtraction;
          cudaSafeCall ( cudaMemcpyFromSymbol (&overestimated, overestimated_global_count, sizeof(overestimated)) );
          cudaSafeCall ( cudaMemcpyFromSymbol (&subtraction, global_count_subtraction, sizeof(subtraction)) );
          data_transfer_finished = !subtraction;

          return overestimated - subtraction;
        }

        static void init_globals()
        {
          const int ZERO = 0;
          cudaSafeCall ( cudaMemcpyToSymbol (global_count_subtraction, &ZERO, sizeof(ZERO)) );
          cudaSafeCall ( cudaMemcpyToSymbol (overestimated_global_count, &ZERO, sizeof(ZERO)) );
        }
      };
    }
  }
}

#endif // TEMPLATED_EXTRACT_CUH

