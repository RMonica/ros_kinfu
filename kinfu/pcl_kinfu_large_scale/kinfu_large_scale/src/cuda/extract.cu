/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 * 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "device.hpp"
#include <iostream>
//#include <boost/graph/buffer_concepts.hpp>

#include "templated_extract.cuh"


namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      struct SlicePointsExtractor
      {
        enum
        {
          MAX_LOCAL_POINTS = 1,
          MIN_X_MARGIN = 0,
          MIN_Y_MARGIN = 0,
          MIN_Z_MARGIN = 0,
        };

          // returns the number of points extracted
        __device__ __forceinline__ int filter(const FullScan6& parent,
          const pcl::gpu::kinfuLS::tsdf_buffer& buffer, int x, int y, int z)
        {
          int W;
          float F = parent.fetch_with_rolling_buffer (x, y, z, W);

          bool in_black_zone =
            ( (x >= minBounds.x && x <= maxBounds.x) ||
              (y >= minBounds.y && y <= maxBounds.y) ||
              ( z >= minBounds.z && z <= maxBounds.z) ) ;
          int local_count = 0;

          if (in_black_zone)
          {
            int W;
            float F = parent.fetch_with_rolling_buffer (x, y, z, W);

            if (W != 0 && F != 1.f && F < 0.98 /*&& F != 0.0f && F > -1.0f*/)
            {
              float4 p;
              p.x = x;
              p.y = y;
              p.z = z;
              p.w = F;
              points[local_count++] = p;
            }
          }

          return local_count;
        }

        __device__ __forceinline__ bool isFull(const FullScan6& parent, unsigned int i)
        {
          return (i >= parent.output_xyz.size);
        }

        __device__ void store(const FullScan6& parent, int offset_storage, int l)
        {
          float x = points[l].x;
          float y = points[l].y;
          float z = points[l].z;
          float i = points[l].w;
          parent.store_point_intensity (x, y, z, i, parent.output_xyz.data, parent.output_intensity.data, offset_storage);
        }

        int knowledge_limit;
        bool edges_only;

        int3 minBounds,maxBounds;

        float4 points[MAX_LOCAL_POINTS];
      };

      struct IncompletePointsExtractor
      {
        enum
        {
          MAX_LOCAL_POINTS = 1,
          MIN_X_MARGIN = 1,
          MIN_Y_MARGIN = 1,
          MIN_Z_MARGIN = 1,
        };
        __device__ IncompletePointsExtractor(int kl,bool eo):
          knowledge_limit(kl), edges_only(eo) {}

          // returns the number of points extracted
        __device__ __forceinline__ int filter(const FullScan6& parent,
          const pcl::gpu::kinfuLS::tsdf_buffer& buffer, int x, int y, int z)
        {
          int W;
          float F = parent.fetch_with_rolling_buffer (x, y, z, W);

          if (W >= knowledge_limit && F > 0.0)
          {
            bool found_unk = false;
            bool found_occ = false;
            float3 unk_tot = make_float3(0.0,0.0,0.0);

            #pragma unroll
            for (int i = 0; i < TOTAL_BEARINGS18; i++)
            {
              int i26 = getBearingIdById18(i);
              int3 b = getBearingById26(i26);
              const float nm = getNormOfBearing(b);

              int Wv;
              float Fv = parent.fetch_with_rolling_buffer (x + b.x,y + b.y,z + b.z,Wv);

              float weight = max(knowledge_limit - Wv,-knowledge_limit);
              if (Wv >= knowledge_limit && Fv < 0.0)
                weight = knowledge_limit;
              unk_tot = make_float3(unk_tot.x + weight * (float(b.x) / nm),
                                    unk_tot.y + weight * (float(b.y) / nm),
                                    unk_tot.z + weight * (float(b.z) / nm));

              if (Wv < knowledge_limit)
              {
                if (isBearingId6(i26)) // restrict to 6-neighborhood for unknown
                  found_unk = true;
              }

              if (Wv >= knowledge_limit)
                if (Fv < 0.0) found_occ = true;
            }

            if ((found_occ == edges_only) && found_unk)
            {
              #pragma unroll
              for (int i = 0; i < TOTAL_BEARINGS26; i++)
                if (!isBearingId18(i))
                {
                  int3 b = getBearingById26(i);
                  const float nm = getNormOfBearing(b);
                  int Wv;
                  const float Fv = parent.fetch_with_rolling_buffer (x + b.x,y + b.y,z + b.z,Wv);
                  float weight = max(knowledge_limit - Wv,-knowledge_limit);
                  if (Wv >= knowledge_limit && Fv < 0.0)
                    weight = knowledge_limit;
                  unk_tot = make_float3(unk_tot.x + weight * (float(b.x) / nm),
                                        unk_tot.y + weight * (float(b.y) / nm),
                                        unk_tot.z + weight * (float(b.z) / nm));
                }
              float3 p; p.x = x; p.y = y; p.z = z;
              float3 n; n.x = -unk_tot.x; n.y = -unk_tot.y; n.z = -unk_tot.z;
              points[0] = p;
              normals[0] = normalized(n);
              return 1;
            }
          }

          return 0;
        }

        enum { TOTAL_BEARINGS26 = 26 };
        __device__ __forceinline__ int3 getBearingById26(int id)
        {
          int3 result;
          int act_id = id < 13 ? id : (id + 1); // avoid (0,0,0)
          result.x = act_id % 3 - 1;
          result.y = (act_id / 3) % 3 - 1;
          result.z = (act_id / 9) % 3 - 1;
          return result;
        }
        __device__ __forceinline__ float getNormOfBearing(int3 b)
        {
          int sn = abs(b.x) + abs(b.y) + abs(b.z);
          if (sn == 1) return 1.0;
          if (sn == 2) return 1.414;
          if (sn == 3) return 1.732;
          return 1.0;
        }
        enum { TOTAL_BEARINGS18 = 18 };
        __device__ __forceinline__ int getBearingIdById18(int id)
        {
          const int ids[TOTAL_BEARINGS18] =
            {1,3,4,5,7,9,10,11,12,13,14,15,16,18,20,21,22,24};
          return ids[id];
        }
        __device__ __forceinline__ bool isBearingId18(int id)
        {
          const int ids[TOTAL_BEARINGS18] =
            {1,3,4,5,7,9,10,11,12,13,14,15,16,18,20,21,22,24};
          #pragma unroll
          for (int i = 0; i < TOTAL_BEARINGS18; i++)
            if (ids[i] == id)
              return true;
          return false;
        }

        enum { TOTAL_BEARINGS6 = 6 };
        __device__ __forceinline__ int getBearingIdById6(int id)
        {
          const int ids[TOTAL_BEARINGS6] = {4,21,12,13,10,15};
          return ids[id];
        }
        __device__ __forceinline__ bool isBearingId6(int id)
        {
          const int ids[TOTAL_BEARINGS6] = {4,21,12,13,10,15};
          #pragma unroll
          for (int i = 0; i < TOTAL_BEARINGS6; i++)
            if (ids[i] == id)
              return true;
          return false;
        }

        __device__ __forceinline__ bool isFull(const FullScan6& parent, unsigned int i)
        {
          return (i >= parent.output_xyz.size);
        }

        __device__ void store(const FullScan6& parent, int offset_storage, int l)
        {
          float x = points[l].x;
          float y = points[l].y;
          float z = points[l].z;
          float nx = normals[l].x;
          float ny = normals[l].y;
          float nz = normals[l].z;
          parent.store_point_normals (x, y, z, nx, ny, nz, parent.output_xyz.data, parent.output_normals.data, offset_storage);
        }

        int knowledge_limit;
        bool edges_only;

        float3 points[MAX_LOCAL_POINTS];
        float3 normals[MAX_LOCAL_POINTS];
        private:
        IncompletePointsExtractor() {}
      };

      __global__ void
      extractSliceKernel (const FullScan6 fs, int3 minBounds, int3 maxBounds)
      {
        SlicePointsExtractor extractor;
        extractor.maxBounds = maxBounds;
        extractor.minBounds = minBounds;
        fs.templatedExtract (extractor);
      }

      __global__ void
      extractIncompletePointsKernel (const FullScan6 fs,bool edges_only)
      {
        IncompletePointsExtractor extractor(5,edges_only);
        fs.templatedExtract (extractor);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      size_t
      extractSliceAsCloud (const PtrStep<short2>& volume, const float3& volume_size, const pcl::gpu::kinfuLS::tsdf_buffer* buffer, 
                           const int shiftX, const int shiftY, const int shiftZ,
                           PtrSz<PointType> output_xyz, PtrSz<float> output_intensities,
                           PtrStep<int> last_data_transfer_matrix, int & data_transfer_finished)
      {
        FullScan6 fs;
        fs.volume = volume;
        fs.cell_size.x = volume_size.x / buffer->voxels_size.x;
        fs.cell_size.y = volume_size.y / buffer->voxels_size.y;
        fs.cell_size.z = volume_size.z / buffer->voxels_size.z;
        fs.output_xyz = output_xyz;
        fs.output_intensity = output_intensities;
        fs.data_transfer_completion_matrix = last_data_transfer_matrix;
        fs.rolling_buffer = *buffer;

        dim3 block (FullScan6::CTA_SIZE_X, FullScan6::CTA_SIZE_Y);
        dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));

        //Compute slice bounds
        int newX = buffer->origin_GRID.x + shiftX;
        int newY = buffer->origin_GRID.y + shiftY;
        int newZ = buffer->origin_GRID.z + shiftZ;

        int3 minBounds, maxBounds;

        //X
        if (newX >= 0)
        {
          minBounds.x = buffer->origin_GRID.x;
          maxBounds.x = newX;    
        }
        else
        {
          minBounds.x = newX + buffer->voxels_size.x;
          maxBounds.x = buffer->origin_GRID.x + buffer->voxels_size.x;
        }
        
        if (minBounds.x > maxBounds.x)
          std::swap (minBounds.x, maxBounds.x);

        //Y
        if (newY >= 0)
        {
          minBounds.y = buffer->origin_GRID.y;
          maxBounds.y = newY;
        }
        else
        {
          minBounds.y = newY + buffer->voxels_size.y;
          maxBounds.y = buffer->origin_GRID.y + buffer->voxels_size.y;
        }

        if(minBounds.y > maxBounds.y)
          std::swap (minBounds.y, maxBounds.y);

        //Z
        if (newZ >= 0)
        {
        minBounds.z = buffer->origin_GRID.z;
        maxBounds.z = newZ;
        }
        else
        {
          minBounds.z = newZ + buffer->voxels_size.z;
          maxBounds.z = buffer->origin_GRID.z + buffer->voxels_size.z;
        }

        if (minBounds.z > maxBounds.z)
          std::swap(minBounds.z, maxBounds.z);

        if (minBounds.x >= 0 && minBounds.x < buffer->origin_GRID.x)
        {
          minBounds.x += buffer->voxels_size.x - buffer->origin_GRID.x;
          maxBounds.x += buffer->voxels_size.x - buffer->origin_GRID.x;
        }
        else
        {
          minBounds.x -= buffer->origin_GRID.x;
          maxBounds.x -= buffer->origin_GRID.x;
        }

        if (minBounds.y >= 0 && minBounds.y < buffer->origin_GRID.y)
        {
          minBounds.y += buffer->voxels_size.y - buffer->origin_GRID.y;
          maxBounds.y += buffer->voxels_size.y - buffer->origin_GRID.y;
        }
        else
        {
          minBounds.y -= buffer->origin_GRID.y;
          maxBounds.y -= buffer->origin_GRID.y;
        }

        if (minBounds.z >= 0 && minBounds.z < buffer->origin_GRID.z)
        {
          minBounds.z += buffer->voxels_size.z - buffer->origin_GRID.z;
          maxBounds.z += buffer->voxels_size.z - buffer->origin_GRID.z;
        }
        else
        {
          minBounds.z -= buffer->origin_GRID.z;
          maxBounds.z -= buffer->origin_GRID.z;
        }

        fs.init_globals();

        // Extraction call
        extractSliceKernel<<<grid, block>>>(fs, minBounds, maxBounds);

        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall ( cudaDeviceSynchronize () );

        int size = fs.get_result_size(data_transfer_finished);
        return min ((int)size, int(output_xyz.size));
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      void
      getDataTransferCompletionMatrixSize(size_t & height, size_t & width)
      {
        dim3 block (FullScan6::CTA_SIZE_X, FullScan6::CTA_SIZE_Y);
        dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));
        width = grid.x;
        height = grid.y;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      void
      getDataTransferCompletionMatrixSize(const int3 & voxels_size,size_t & height, size_t & width)
      {
        dim3 block (FullScan6::CTA_SIZE_X, FullScan6::CTA_SIZE_Y);
        dim3 grid (divUp (voxels_size.x, block.x), divUp (voxels_size.y, block.y));
        width = grid.x;
        height = grid.y;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      size_t
      extractIncompletePointsAsCloud (const PtrStep<short2>& volume, const float3& volume_size,
                           const pcl::gpu::kinfuLS::tsdf_buffer* buffer,const bool edges_only,
                           PtrSz<PointType> output_xyz, PtrSz<float4> output_normals,
                           PtrStep<int> last_data_transfer_matrix, int & data_transfer_finished)
      {
        FullScan6 fs;
        fs.volume = volume;
        fs.cell_size.x = volume_size.x / buffer->voxels_size.x;
        fs.cell_size.y = volume_size.y / buffer->voxels_size.y;
        fs.cell_size.z = volume_size.z / buffer->voxels_size.z;
        fs.output_xyz = output_xyz;
        fs.output_normals = output_normals;
        fs.data_transfer_completion_matrix = last_data_transfer_matrix;
        fs.rolling_buffer = *buffer;

        dim3 block (FullScan6::CTA_SIZE_X, FullScan6::CTA_SIZE_Y);
        dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));

        fs.init_globals();

        // Extraction call
        extractIncompletePointsKernel<<<grid, block>>>(fs, edges_only);

        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall ( cudaDeviceSynchronize () );

        int size = fs.get_result_size(data_transfer_finished);
        return min (size, int(output_xyz.size));
      }
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      template<typename NormalType>
      struct ExtractNormals
      {
        float3 cell_size;
        PtrStep<short2> volume;
        PtrSz<PointType> points;

        mutable NormalType* output;

        __device__ __forceinline__ float
        readTsdf (int x, int y, int z) const
        {
          return unpack_tsdf (volume.ptr (VOLUME_Y * z + y)[x]);
        }

        __device__ __forceinline__ float3
        fetchPoint (int idx) const
        {
          PointType p = points.data[idx];
          return make_float3 (p.x, p.y, p.z);
        }
        __device__ __forceinline__ void
        storeNormal (int idx, float3 normal) const
        {
          NormalType n;
          n.x = normal.x; n.y = normal.y; n.z = normal.z;
          output[idx] = n;
        }

        __device__ __forceinline__ int3
        getVoxel (const float3& point) const
        {
          int vx = __float2int_rd (point.x / cell_size.x);        // round to negative infinity
          int vy = __float2int_rd (point.y / cell_size.y);
          int vz = __float2int_rd (point.z / cell_size.z);

          return make_int3 (vx, vy, vz);
        }

        __device__ __forceinline__ void
        operator () () const
        {
          int idx = threadIdx.x + blockIdx.x * blockDim.x;

          if (idx >= points.size)
            return;
          const float qnan = numeric_limits<float>::quiet_NaN ();
          float3 n = make_float3 (qnan, qnan, qnan);

          float3 point = fetchPoint (idx);
          int3 g = getVoxel (point);

          if (g.x > 1 && g.y > 1 && g.z > 1 && g.x < VOLUME_X - 2 && g.y < VOLUME_Y - 2 && g.z < VOLUME_Z - 2)
          {
            float3 t;

            t = point;
            t.x += cell_size.x;
            float Fx1 = interpolateTrilineary (t);

            t = point;
            t.x -= cell_size.x;
            float Fx2 = interpolateTrilineary (t);

            n.x = (Fx1 - Fx2);

            t = point;
            t.y += cell_size.y;
            float Fy1 = interpolateTrilineary (t);

            t = point;
            t.y -= cell_size.y;
            float Fy2 = interpolateTrilineary (t);

            n.y = (Fy1 - Fy2);

            t = point;
            t.z += cell_size.z;
            float Fz1 = interpolateTrilineary (t);

            t = point;
            t.z -= cell_size.z;
            float Fz2 = interpolateTrilineary (t);

            n.z = (Fz1 - Fz2);

            n = normalized (n);
          }
          storeNormal (idx, n);
        }

        __device__ __forceinline__ float
        interpolateTrilineary (const float3& point) const
        {
          int3 g = getVoxel (point);

  /*
          //OLD CODE
          float vx = (g.x + 0.5f) * cell_size.x;
          float vy = (g.y + 0.5f) * cell_size.y;
          float vz = (g.z + 0.5f) * cell_size.z;

          if (point.x < vx) g.x--;
          if (point.y < vy) g.y--;
          if (point.z < vz) g.z--;

          //float a = (point.x - (g.x + 0.5f) * cell_size.x) / cell_size.x;
          //float b = (point.y - (g.y + 0.5f) * cell_size.y) / cell_size.y;
          //float c = (point.z - (g.z + 0.5f) * cell_size.z) / cell_size.z;
          float a =  point.x/ cell_size.x - (g.x + 0.5f);
          float b =  point.y/ cell_size.y - (g.y + 0.5f);
          float c =  point.z/ cell_size.z - (g.z + 0.5f);
  */
          //NEW CODE
                  float a = point.x/ cell_size.x - (g.x + 0.5f); if (a<0) { g.x--; a+=1.0f; };
          float b = point.y/ cell_size.y - (g.y + 0.5f); if (b<0) { g.y--; b+=1.0f; };
          float c = point.z/ cell_size.z - (g.z + 0.5f); if (c<0) { g.z--; c+=1.0f; };

          float res = (1 - a) * ( 
                                  (1 - b) * ( readTsdf (g.x + 0, g.y + 0, g.z + 0) * (1 - c) +
                                              readTsdf (g.x + 0, g.y + 0, g.z + 1) *    c  )
                                          + b * ( readTsdf (g.x + 0, g.y + 1, g.z + 0) * (1 - c) +
                                                  readTsdf (g.x + 0, g.y + 1, g.z + 1) *    c  )
                          ) + a * (
                                  (1 - b) * ( readTsdf (g.x + 1, g.y + 0, g.z + 0) * (1 - c) +
                                              readTsdf (g.x + 1, g.y + 0, g.z + 1) *    c  )
                                          + b * ( readTsdf (g.x + 1, g.y + 1, g.z + 0) * (1 - c) +
                                                  readTsdf (g.x + 1, g.y + 1, g.z + 1) *    c  )
                          );

          return res;
        }
      };

      template<typename NormalType>
      __global__ void
      extractNormalsKernel (const ExtractNormals<NormalType> en) {
        en ();
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      template<typename NormalType> void
      extractNormals (const PtrStep<short2>& volume, const float3& volume_size, 
                                  const PtrSz<PointType>& points, NormalType* output)
      {
        ExtractNormals<NormalType> en;
        en.volume = volume;
        en.cell_size.x = volume_size.x / VOLUME_X;
        en.cell_size.y = volume_size.y / VOLUME_Y;
        en.cell_size.z = volume_size.z / VOLUME_Z;
        en.points = points;
        en.output = output;

        dim3 block (256);
        dim3 grid (divUp (points.size, block.x));

        extractNormalsKernel<<<grid, block>>>(en);
        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());
      }

      template void extractNormals<PointType>(const PtrStep<short2>&volume, const float3 &volume_size, const PtrSz<PointType>&input, PointType * output);
      template void extractNormals<float8>(const PtrStep<short2>&volume, const float3 &volume_size, const PtrSz<PointType>&input, float8 * output);

    }
  }
}
